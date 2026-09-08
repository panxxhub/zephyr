#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Compile actual driver functions against fake hardware; restored defects must assert."""

import os
import re
import resource
import signal
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BUILD = ROOT / '.scratch' / 'gem-host'
BUILD.mkdir(parents=True, exist_ok=True)
os.environ['TMPDIR'] = str(BUILD)
resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def function(source, name):
    match = re.search(r'(?:static )?(?:inline )?[\w *]+\b' + name + r'\([^;]*?\)\n\{', source)
    if match is None:
        raise AssertionError(f'missing production function {name}')
    end = source.index('\n}', match.end()) + 2
    return source[match.start() : end].strip()


def macros(source):
    return '\n'.join(
        line
        for line in source.splitlines()
        if line.startswith('#define ETH_XLNX_GEM_') and '(' not in line.split()[1]
    )


def compile_run(source, name, cases, reverse=False):
    path = BUILD / f'{name}.c'
    exe = BUILD / name
    path.write_text(source)
    subprocess.run(
        [
            os.environ.get('CC', 'cc'),
            '-std=c11',
            '-g',
            '-O1',
            '-fno-pie',
            '-no-pie',
            '-Wall',
            '-Wextra',
            '-Werror',
            '-Wno-unused-parameter',
            '-Wno-unused-function',
            '-Wno-unused-variable',
            '-Wno-pointer-to-int-cast',
            '-Wno-sign-compare',
            '-fsanitize=undefined',
            '-pthread',
            str(path),
            '-o',
            str(exe),
        ],
        check=True,
    )
    for case in cases:
        result = subprocess.run([str(exe), str(case)], capture_output=True, timeout=5)
        expected = -signal.SIGABRT if reverse else 0
        if result.returncode != expected:
            raise AssertionError(
                f'{name} T{case}: exit {result.returncode}, expected {expected}\n'
                + result.stdout.decode()
                + result.stderr.decode()
            )
        print(f'{name}: T{case} {"restored defect rejected" if reverse else "PASS"}', flush=True)


def mutation(source, old, new):
    assert source.count(old) == 1, (old, source.count(old))
    return source.replace(old, new)


def gem():
    source = (ROOT / 'drivers/ethernet/eth_xlnx_gem.c').read_text()
    header = (ROOT / 'drivers/ethernet/eth_xlnx_gem_priv.h').read_text()
    public = (ROOT / 'include/zephyr/drivers/ethernet/eth_xlnx_gem.h').read_text()
    names = [
        'eth_xlnx_gem_configure_buffers',
        'eth_xlnx_gem_request_recovery',
        'eth_xlnx_gem_recovery_work',
        'eth_xlnx_gem_handle_rx_pending',
        'eth_xlnx_gem_handle_tx_done',
        'eth_xlnx_gem_send',
        'eth_xlnx_gem_isr',
        'eth_xlnx_gem_phy_cb',
        'eth_xlnx_gem_start_device',
    ]
    functions = '\n\n'.join(function(source, n) for n in names)
    start = source.index('#define GEM_RX_IRQS')
    end = source.index('\n\n', start)
    functions = source[start:end] + '\n' + functions
    diagnostics = public[
        public.index('struct eth_xlnx_gem_diagnostics {') : public.index('\n};') + 3
    ]
    host = (HERE / 'gem_host.c').read_text().replace('/* DIAGNOSTICS */', diagnostics)
    host = host.replace('/* MACROS */', macros(header) + '\n' + macros(public))
    host = host.replace('/* FUNCTIONS */', functions)
    compile_run(host, 'gem', [1, 2, 3, 6, 7, 8, 9, 10, 11, 13])
    # Restore the previous pacing and RX-pressure reset policy together.
    previous = mutation(
        host,
        'k_work_schedule(&dev_data->rx_pend_work, K_NO_WAIT);',
        'k_work_schedule(&dev_data->rx_pend_work, K_MSEC(2));',
    )
    previous = mutation(
        previous,
        'k_work_schedule(&data->rx_pend_work, shed_load ? K_MSEC(2) : K_NO_WAIT);',
        'k_work_schedule(&data->rx_pend_work, K_MSEC(2));',
    )
    previous = mutation(
        previous,
        '(reg_val & GEM_RECOVERY_IRQS) != 0U',
        '(reg_val & (GEM_RECOVERY_IRQS | 0x404)) != 0U',
    )
    compile_run(previous, 'gem_reverse_previous_rx_policy', [13], reverse=True)
    mutants = [
        (
            'masked_rx_status',
            13,
            '~sys_read32(DEVICE_MMIO_NAMED_GET(dev, mac) + ETH_XLNX_GEM_IMR_OFFSET)',
            'UINT32_MAX',
        ),
        (
            'unconditional_pacing',
            13,
            'k_work_schedule(&data->rx_pend_work, shed_load ? K_MSEC(2) : K_NO_WAIT);',
            'k_work_schedule(&data->rx_pend_work, K_MSEC(2));',
        ),
        (
            'pressure_resets_tx',
            13,
            '(reg_val & GEM_RECOVERY_IRQS) != 0U',
            '(reg_val & (GEM_RECOVERY_IRQS | 0x404)) != 0U',
        ),
        (
            'pressure_status',
            13,
            'sys_write32(BIT(0) | BIT(1) | BIT(2), base + ETH_XLNX_GEM_RXSR_OFFSET);',
            'sys_write32(BIT(1), base + ETH_XLNX_GEM_RXSR_OFFSET);',
        ),
        (
            'pressure_irq_enable',
            13,
            'sys_write32(GEM_RX_IRQS, base + ETH_XLNX_GEM_IER_OFFSET);',
            'sys_write32(ETH_XLNX_GEM_IXR_FRAME_RX_BIT, base + ETH_XLNX_GEM_IER_OFFSET);',
        ),
        (
            'drop_log',
            9,
            'data->diagnostics.rx_dropped += receive_result < 0;',
            'data->diagnostics.rx_dropped += receive_result < 0; LOG_ERR("drop");',
        ),
        (
            'empty_pool_copy',
            9,
            'if (pkt != NULL) {\n#ifdef CONFIG_DCACHE',
            'if (true) {\n#ifdef CONFIG_DCACHE',
        ),
        (
            'busy_wait',
            11,
            'data->diagnostics.recoveries++;',
            'data->diagnostics.recoveries++; k_busy_wait(1000);',
        ),
        (
            'rx_irq_pause',
            9,
            'k_work_schedule(&data->rx_pend_work, shed_load ? K_MSEC(2) : K_NO_WAIT);',
            'sys_write32(ETH_XLNX_GEM_IXR_FRAME_RX_BIT, base + ETH_XLNX_GEM_IER_OFFSET);'
            'k_work_schedule(&data->rx_pend_work, shed_load ? K_MSEC(2) : K_NO_WAIT);',
        ),
        (
            'rx_pause',
            9,
            'k_work_schedule(&data->rx_pend_work, shed_load ? K_MSEC(2) : K_NO_WAIT);',
            'k_work_schedule(&data->rx_pend_work, K_NO_WAIT);',
        ),
        ('rx_frames', 9, 'uint8_t frames = 8;', 'uint8_t frames = 32;'),
        (
            'recovery_pause',
            11,
            'k_work_schedule(&data->recovery_work, K_MSEC(2));',
            'k_work_schedule(&data->recovery_work, K_NO_WAIT);',
        ),
        ('trace_orphan', 10, 'data->diagnostics.rx_orphans++;', 'break;'),
        ('orphan', 1, 'data->diagnostics.rx_orphans++;', 'break;'),
        ('error_irq', 1, '(reg_val & GEM_RECOVERY_IRQS) != 0U', 'false'),
        (
            'ownership',
            3,
            '(sys_read32((uintptr_t)&bd->addr) &\n\t\t\t     ETH_XLNX_GEM_RX_BD_USED_BIT) == 0U',
            'false',
        ),
        ('eof_bound', 3, 'count < budget', 'true'),
        ('drain_bound', 3, 'budget -= count;', 'budget = cfg->rx_bd_count; frames++;'),
        ('length', 3, 'length > NET_ETH_MAX_FRAME_SIZE', 'length > UINT32_MAX'),
        (
            'timeout',
            2,
            'eth_xlnx_gem_request_recovery(dev, ETH_XLNX_GEM_RECOVERY_TX_TIMEOUT);',
            '(void)dev;',
        ),
        ('tx_ownership', 2, '(head & ETH_XLNX_GEM_TX_BD_USED_BIT) == 0U', 'false'),
        (
            'tx_accounting',
            2,
            'outstanding = cfg->tx_bd_count - data->tx_bd_ring.free_bds;',
            'outstanding = cfg->tx_bd_count;',
        ),
        ('tx_error', 2, '(ctrl & GEM_TX_ERRORS) != 0U', 'false'),
        (
            'mac_speed',
            6,
            'state->is_up && state->speed != LINK_HALF_10BASE',
            'false && state->speed != LINK_HALF_10BASE',
        ),
        ('tx_cache', 7, 'sys_cache_data_flush_and_invd_range(buffer, cfg->tx_buffer_size);', ''),
        (
            'rx_cache',
            7,
            'sys_cache_data_flush_and_invd_range(dev_data->first_rx_buffer,\n'
            '\t\t\t\t\t dev_conf->rx_bd_count * dev_conf->rx_buffer_size);',
            '',
        ),
    ]
    # Capacity validation is tested independently from the MTU ceiling.
    mutants[next(i for i, item in enumerate(mutants) if item[0] == 'length')] = (
        'length',
        3,
        'length + cfg->hw_rx_buffer_offset > count * cfg->rx_buffer_size',
        'false',
    )
    for name, case, old, new in mutants:
        changed = mutation(host, old, new)
        if name == 'eof_bound':
            changed = mutation(
                changed,
                'count != 0U && (ctrl & ETH_XLNX_GEM_RX_BD_START_OF_FRAME_BIT) != 0U',
                'false',
            )
        compile_run(changed, f'gem_reverse_{name}', [case], reverse=True)


def phy():
    source = (ROOT / 'drivers/ethernet/phy/phy_motorcomm_yt8531.c').read_text()
    names = [
        'mc_yt8531_soft_reset',
        'mc_yt8531_get_link_speed_stat_reg',
        'mc_yt8531_read_live_link_state',
        'update_link_state',
        'invoke_link_cb',
        'monitor_work_handler',
        'mc_yt8531_get_link_state',
        'mc_yt8531_link_cb_set',
    ]
    body = '\n\n'.join(function(source, name) for name in names)
    host = (HERE / 'phy_host.c').read_text().replace('/* FUNCTIONS */', body)
    compile_run(host, 'phy', [6, 60])
    mutants = [
        ('reset', 60, 'return -ETIMEDOUT;', 'return 0;'),
        ('uninitialized', 6, '*state = (struct phy_link_state){0};', ''),
        (
            'second_read',
            6,
            '/* Caller holds the recursive PHY lock.',
            'mc_yt8531_get_link_state(dev, &state);\n\t/* Caller holds the recursive PHY lock.',
        ),
        (
            'getter_consumes',
            6,
            'ret = mc_yt8531_read_live_link_state(dev, state);',
            'ret = mc_yt8531_read_live_link_state(dev, state);\n'
            '\tif (ret == 0) { data->notified = *state; }',
        ),
        ('invalid_speed', 6, 'if (state->speed == 0)', 'if (false)'),
    ]
    for name, case, old, new in mutants:
        compile_run(mutation(host, old, new), f'phy_reverse_{name}', [case], reverse=True)


def mdio():
    source = (ROOT / 'drivers/ethernet/mdio/mdio_xlnx_gem.c').read_text()
    names = [
        'xlnx_gem_mdio_is_idle',
        'xlnx_gem_mdio_poll_idle',
        'xlnx_gem_mdio_transfer',
        'xlnx_gem_mdio_read',
        'xlnx_gem_mdio_write',
        'xlnx_gem_mdio_read_c45',
        'xlnx_gem_mdio_write_c45',
    ]
    body = '\n\n'.join(function(source, name) for name in names)
    body = declaration(source, 'xlnx_gem_mdio_data') + '\n' + macros(source) + '\n' + body
    host = (HERE / 'mdio_host.c').read_text().replace('/* FUNCTIONS */', body)
    config = (ROOT / 'drivers/ethernet/mdio/Kconfig.xlnx_gem').read_text()
    default = re.search(
        r'config MDIO_XLNX_GEM_IDLE_TIMEOUT_US.*?default (\d+)', config, re.S
    ).group(1)
    host = host.replace(
        '#define CONFIG_MDIO_XLNX_GEM_IDLE_TIMEOUT_US 1000',
        f'#define CONFIG_MDIO_XLNX_GEM_IDLE_TIMEOUT_US {default}',
    )
    compile_run(host, 'mdio', [5])
    compile_run(
        mutation(host, 'runtime->idle_wait_max_us = runtime->idle_wait_last_us;', ''),
        'mdio_reverse_wait_measurement',
        [5],
        reverse=True,
    )
    unlocked = host.replace('int ret = k_mutex_lock(&runtime->lock, K_MSEC(10));', 'int ret = 0;')
    unlocked = unlocked.replace('k_mutex_unlock(&runtime->lock);', '')
    compile_run(unlocked, 'mdio_reverse_lock', [5], reverse=True)
    split = host.replace(
        'if (ret == 0) {\n\t\tret = xlnx_gem_mdio_transfer',
        'if (ret == 0) {\n\t\tk_mutex_unlock(&runtime->lock);\n'
        '\t\tk_mutex_lock(&runtime->lock, K_MSEC(10));\n'
        '\t\tret = xlnx_gem_mdio_transfer',
    )
    assert split != host
    compile_run(split, 'mdio_reverse_c45', [5], reverse=True)
    compile_run(
        mutation(host, 'k_usleep(50);', 'ticks += 50;'), 'mdio_reverse_busy_wait', [5], reverse=True
    )
    compile_run(
        mutation(
            host,
            '#define CONFIG_MDIO_XLNX_GEM_IDLE_TIMEOUT_US 1000',
            '#define CONFIG_MDIO_XLNX_GEM_IDLE_TIMEOUT_US 1000000',
        ),
        'mdio_reverse_deadline',
        [5],
        reverse=True,
    )


def phy_io():
    source = (ROOT / 'drivers/ethernet/phy/phy_motorcomm_yt8531.c').read_text()
    names = [
        'mc_yt8531_read',
        'mc_yt8531_write',
        'mc_yt8531_modify',
        'mc_yt8531_read_ext',
        'mc_yt8531_write_ext',
        'mc_yt8531_modify_ext',
    ]
    body = '\n\n'.join(function(source, name) for name in names)
    host = (HERE / 'phy_io_host.c').read_text().replace('/* FUNCTIONS */', body)
    compile_run(host, 'phy_io', [5])
    changed = host.replace('int ret = k_mutex_lock(&runtime->lock, K_MSEC(10));', 'int ret = 0;')
    changed = changed.replace('k_mutex_unlock(&runtime->lock);', '')
    compile_run(changed, 'phy_reverse_page_lock', [5], reverse=True)


def declaration(source, name):
    start = source.index(f'struct {name} {{')
    return source[start : source.index('\n};', start) + 3]


def mmio_macro(source, name):
    lines = source.splitlines()
    index = next(i for i, line in enumerate(lines) if line.startswith(f'#define {name}('))
    result = [lines[index]]
    while result[-1].endswith('\\'):
        index += 1
        result.append(lines[index])
    return '\n'.join(result)


def bringup():
    gem_header = (ROOT / 'drivers/ethernet/eth_xlnx_gem_priv.h').read_text()
    public = (ROOT / 'include/zephyr/drivers/ethernet/eth_xlnx_gem.h').read_text()
    mmio_header = (ROOT / 'include/zephyr/sys/device_mmio.h').read_text()
    mdio_source = (ROOT / 'drivers/ethernet/mdio/mdio_xlnx_gem.c').read_text()
    phy_source = (ROOT / 'drivers/ethernet/phy/phy_motorcomm_yt8531.c').read_text()
    mii = (ROOT / 'include/zephyr/net/mii.h').read_text()
    autoneg = (ROOT / 'drivers/ethernet/phy/phy_mii.h').read_text()
    declarations = '\n'.join(
        [
            mmio_macro(mmio_header, 'DEVICE_MMIO_NAMED_RAM'),
            mmio_macro(mmio_header, 'DEVICE_MMIO_RAM_PTR'),
            mmio_macro(mmio_header, 'DEVICE_MMIO_GET'),
            declaration(public, 'eth_xlnx_gem_diagnostics'),
            declaration(gem_header, 'eth_xlnx_gem_dev_data'),
            declaration(mdio_source, 'xlnx_gem_mdio_data'),
            declaration(phy_source, 'mc_yt8531_config'),
            declaration(phy_source, 'mc_yt8531_data'),
            macros(mdio_source),
            mii.replace('#include <zephyr/sys/util_macro.h>', ''),
            '\n'.join(
                line
                for line in phy_source.splitlines()
                if line.startswith('#define ')
                and not line.startswith(('#define DT_', '#define MC_'))
            ),
        ]
    )
    mdio_names = [
        'xlnx_gem_mdio_is_idle',
        'xlnx_gem_mdio_poll_idle',
        'xlnx_gem_mdio_transfer',
        'xlnx_gem_mdio_read',
        'xlnx_gem_mdio_write',
        'xlnx_gem_mdio_initialize',
    ]
    phy_start = phy_source.index('static int mc_yt8531_read(')
    phy_end = phy_source.index('static DEVICE_API')
    host = (HERE / 'bringup_host.c').read_text()
    host = host.replace(
        '/* DECLARATIONS: production MMIO macros, parent layout, MDIO and PHY '
        'structs/constants. */',
        declarations,
    )
    host = host.replace(
        '/* MDIO FUNCTIONS */', '\n'.join(function(mdio_source, name) for name in mdio_names)
    )
    host = host.replace(
        '/* AUTONEG FUNCTIONS */',
        '\n'.join(
            function(autoneg, name)
            for name in ['phy_mii_set_anar_reg', 'phy_mii_set_c1kt_reg', 'phy_mii_cfg_link_autoneg']
        ),
    )
    host = host.replace('/* PHY FUNCTIONS */', phy_source[phy_start:phy_end])
    compile_run(host, 'h2plus_bringup', [8])
    # Restore the exact 23fe8c0d layout: recovery fields precede the MMIO slots.
    fields = '\tDEVICE_MMIO_NAMED_RAM(mac);\n\tDEVICE_MMIO_NAMED_RAM(clkc);\n'
    changed = mutation(host, fields, '')
    changed = mutation(changed, '\tint tx_result;', '\tint tx_result;\n' + fields)
    compile_run(changed, 'h2plus_reverse_mmio_layout', [8], reverse=True)


def coap_log():
    source = (ROOT / 'subsys/net/lib/coap/coap.c').read_text()
    server = (ROOT / 'subsys/net/lib/coap/coap_server.c').read_text()
    header = (ROOT / 'include/zephyr/net/coap.h').read_text()
    names = [
        'option_header_get_delta',
        'option_header_get_len',
        'read_u8',
        'read_be16',
        'read',
        'decode_delta',
        'parse_option',
        'coap_packet_parse',
    ]
    host = (HERE / 'coap_log_host.c').read_text()
    logging = (ROOT / 'include/zephyr/logging/log.h').read_text()
    host = host.replace(
        '/* NATIVE RATELIMIT MACROS */',
        '\n'.join(
            mmio_macro(logging, name)
            for name in ['_LOG_RATELIMIT_CORE', '_LOG_RATELIMIT_LVL', 'LOG_ERR_RATELIMIT_RATE']
        ),
    )
    declarations = '\n'.join(
        [declaration(header, 'coap_packet'), declaration(header, 'coap_option')]
    )
    declarations += '\n' + '\n'.join(
        line
        for line in source.splitlines()
        if line.startswith(
            ('#define COAP_OPTION_', '#define COAP_MARKER', '#define BASIC_HEADER_SIZE')
        )
    )
    host = host.replace('/* DECLARATIONS */', declarations)
    host = host.replace('/* PARSER FUNCTIONS */', '\n'.join(function(source, n) for n in names))
    # Extract the real socket receive/parse/error-return prefix. Successful
    # dispatch is outside this malformed-datagram test's scope.
    process = function(server, 'coap_server_process')
    end = process.index('\n\t/* RFC 7252 Section 5.4.1:')
    host = host.replace('/* SERVER REJECTION PATH */', process[:end] + '\n\treturn 0;\n}')
    compile_run(host, 'coap_slow_log', [12])
    compile_run(
        re.sub(r'(#define TEST_DEBUG_LOGGING\s+)0', r'\g<1>1', host),
        'coap_slow_log_debug',
        [12],
    )
    for name, old, new in [
        (
            'server',
            'LOG_ERR_RATELIMIT_RATE(1000, "Failed To parse coap message',
            'LOG_ERR("Failed To parse coap message',
        ),
        (
            'option',
            'LOG_ERR_RATELIMIT_RATE(1000, "%u is > sizeof(coap_option',
            'NET_ERR("%u is > sizeof(coap_option',
        ),
    ]:
        compile_run(mutation(host, old, new), f'coap_reverse_{name}_log', [12], reverse=True)


if __name__ == '__main__':
    gem()
    phy()
    mdio()
    phy_io()
    bringup()
    coap_log()
