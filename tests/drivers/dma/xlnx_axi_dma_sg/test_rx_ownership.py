# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Execute driver RX admission/lifetime code with fake hardware; kill regressions."""

import re
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
SOURCE = ROOT / 'drivers/dma/dma_xlnx_axi_dma_sg.c'


def function(source, name):
    match = re.search(r'^(?:static )?(?:int|void) ' + name + r'\([^;]*?\n\{', source, re.M)
    start = match.start()
    end = source.index('\n}', match.end()) + 2
    return source[start:end]


STUB = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define CH_RX                1
#define NUM_CHANNELS         2
#define CH_TX                0
#define MEMORY_TO_PERIPHERAL 1
#define PERIPHERAL_TO_MEMORY 2
#define REG_DMACR            0
#define REG_DMASR            1
#define DMACR_RS             1
#define DMASR_HALTED         1
#define LOG_ERR(...)
typedef int atomic_t;
static bool atomic_cas(atomic_t *p, int old, int next)
{
	return __sync_bool_compare_and_swap(p, old, next);
}
static void atomic_clear(atomic_t *p)
{
	*p = 0;
}
static int atomic_get(atomic_t *p)
{
	return *p;
}
static void atomic_set(atomic_t *p, int n)
{
	*p = n;
}
struct device {
	void *data;
};
struct dma_config {
	int channel_direction;
	int cyclic;
};
struct dma_xlnx_sg_rx_stream_cfg {
	unsigned bd_bytes, irq_threshold;
	void *callback;
};
struct dma_xlnx_sg_chan {
	unsigned num_bds;
	atomic_t rx_owner;
	bool rx_stream_active;
	void *rx_stream_callback, *rx_stream_error_callback, *rx_stream_user_data;
	atomic_t rx_windows_ready, rx_error_pending;
	int rx_stream_work, rx_stream_work_sync;
};
struct dma_xlnx_sg_data {
	struct dma_xlnx_sg_chan ch[2];
};
static unsigned prepares, configures, stops, cancels;
static int start_error;
static unsigned halted = 1, resets;
static int reset_error;
static uint32_t chan_read(const struct device *dev, unsigned ch, unsigned reg)
{
	(void)dev;
	(void)ch;
	return reg == REG_DMASR ? halted : 0;
}
static int do_soft_reset(const struct device *dev, unsigned ch)
{
	(void)dev;
	(void)ch;
	resets++;
	halted = 1;
	return reset_error;
}
static size_t buf_size(const struct device *d, int c)
{
	(void)d;
	(void)c;
	return 8192;
}
static void dma_xlnx_sg_prepare_rx_stream(const struct device *d,
					  const struct dma_xlnx_sg_rx_stream_cfg *c)
{
	struct dma_xlnx_sg_data *s = d->data;
	assert(s->ch[1].rx_owner == 2);
	prepares++;
	s->ch[1].rx_stream_callback = c->callback;
}
static int dma_xlnx_sg_reconfigure_rx(const struct device *d, unsigned b, unsigned t)
{
	(void)d;
	(void)b;
	(void)t;
	return start_error;
}
static int dma_xlnx_sg_stop(const struct device *d, int c)
{
	struct dma_xlnx_sg_data *s = d->data;
	(void)c;
	assert(s->ch[1].rx_owner == 2);
	stops++;
	return 0;
}
static struct dma_xlnx_sg_data state;
static int k_work_cancel_sync(int *w, int *s)
{
	(void)w;
	(void)s;
	assert(state.ch[1].rx_owner == 2);
	cancels++;
	return 0;
}
'''
TEST = r'''
int main(void)
{
	state.ch[1].num_bds = 4;
	struct device dev = {.data = &state};
	struct dma_xlnx_sg_rx_stream_cfg cfg = {
		.bd_bytes = 1024, .irq_threshold = 1, .callback = &state};
	struct dma_config finite = {.channel_direction = PERIPHERAL_TO_MEMORY};
	assert(dma_xlnx_sg_reserve_rx(&dev) == 0);
	assert(dma_xlnx_sg_reserve_rx(&dev) == -EBUSY);
	assert(dma_xlnx_sg_start_rx_stream(&dev, &cfg) == -EBUSY);
	assert(prepares == 0);
	assert(dma_xlnx_sg_config(&dev, CH_RX, &finite) == 0);
	dma_xlnx_sg_stop_rx_stream(&dev);
	assert(stops == 0 && state.ch[1].rx_owner == 1);
	dma_xlnx_sg_release_rx(&dev);
	assert(dma_xlnx_sg_start_rx_stream(&dev, &cfg) == 0);
	assert(dma_xlnx_sg_reserve_rx(&dev) == -EBUSY);
	assert(dma_xlnx_sg_config(&dev, CH_RX, &finite) == -EBUSY);
	assert(configures == 1);
	dma_xlnx_sg_release_rx(&dev);
	assert(state.ch[1].rx_owner == 2);
	dma_xlnx_sg_stop_rx_stream(&dev);
	assert(stops == 1 && cancels == 1 && state.ch[1].rx_owner == 0);
	assert(dma_xlnx_sg_reserve_rx(&dev) == 0);
	dma_xlnx_sg_release_rx(&dev);
	halted = 0;
	assert(dma_xlnx_sg_reserve_rx(&dev) == 0);
	assert(resets == 1);
	dma_xlnx_sg_release_rx(&dev);
	halted = 0;
	reset_error = -EIO;
	assert(dma_xlnx_sg_reserve_rx(&dev) == -EIO);
	assert(state.ch[1].rx_owner == 0);
	reset_error = 0;
	start_error = -EIO;
	assert(dma_xlnx_sg_start_rx_stream(&dev, &cfg) == -EIO);
	assert(dma_xlnx_sg_reserve_rx(&dev) == 0);
	return 0;
}
'''


def main():
    source = SOURCE.read_text()
    # Admission prefix is before callback/descriptor/hardware writes.
    config = function(source, 'dma_xlnx_sg_config').split('\t/* Store callback */')[0]
    bodies = config + '\tconfigures++; return 0;\n}\n'
    for name in (
        'dma_xlnx_sg_reserve_rx',
        'dma_xlnx_sg_release_rx',
        'dma_xlnx_sg_start_rx_stream',
        'dma_xlnx_sg_stop_rx_stream',
    ):
        bodies += function(source, name) + '\n'
    mutations = [
        ('atomic_cas(&data->ch[CH_RX].rx_owner, 0, 1) ? 0 : -EBUSY', '0'),
        ('!atomic_cas(&ch->rx_owner, 0, 2)', 'ch->rx_stream_active'),
        ('channel == CH_RX && atomic_get(&ch->rx_owner) == 2', 'false'),
        ('atomic_clear(&ch->rx_owner);', '(void)0;'),
        ('ret = do_soft_reset(dev, CH_RX);', 'ret = 0;'),
    ]
    variants = [bodies]
    for old, new in mutations:
        assert old in bodies
        variants.append(bodies.replace(old, new))
    with tempfile.TemporaryDirectory() as directory:
        directory = Path(directory)
        for i, body in enumerate(variants):
            cfile = directory / 'test.c'
            cfile.write_text(STUB + body + TEST)
            exe = directory / 'test'
            subprocess.run(['cc', '-std=gnu11', '-O2', str(cfile), '-o', str(exe)], check=True)
            run = subprocess.run([str(exe)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            assert (run.returncode == 0) == (i == 0), f'variant {i}: {run.returncode}'
    print('RX reservation/stream exclusion and release: PASS; 5 reverse mutations: RED')


if __name__ == '__main__':
    main()
