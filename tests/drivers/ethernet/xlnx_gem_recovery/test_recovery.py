#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0
"""Compile actual driver functions against fake hardware; restored defects must assert."""

import hashlib
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


def compile_run(source, name, cases, reverse=False, companions=()):
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
            '-Wno-int-to-pointer-cast',
            '-Wno-sign-compare',
            '-fsanitize=undefined',
            '-pthread',
            str(path),
            *map(str, companions),
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
    orphan_old = (
        '\t\t\tLOG_ERR("%s unexpected missing SOF bit in RX BD [%u]",\n'
        '\t\t\t\tdev->name, first_bd_idx);\n'
        '\t\t\tbreak;'
    )
    orphan_new = (
        '\t\t\tLOG_ERR_RATELIMIT_RATE(1000, "%s unexpected missing SOF bit in RX BD [%u]",\n'
        '\t\t\t\t\t       dev->name, first_bd_idx);\n'
        '\t\t\t/* Recycle only this CPU-owned orphan, preserving address and wrap. */\n'
        '\t\t\tsys_write32(0U, reg_ctrl);\n'
        '\t\t\tbarrier_dmem_fence_full();\n'
        '\t\t\treg_val = sys_read32(reg_addr) & ~ETH_XLNX_GEM_RX_BD_USED_BIT;\n'
        '\t\t\tsys_write32(reg_val, reg_addr);\n'
        '\t\t\tdev_data->rx_bd_ring.next_to_process =\n'
        '\t\t\t\t(first_bd_idx + 1U) % dev_conf->rx_bd_count;\n'
        '\t\t\tcontinue;'
    )
    restored = mutation(source, orphan_new, orphan_old)
    normal = re.sub(
        r'LOG_(?:ERR|WRN)(?:_RATELIMIT_RATE)?\(.*?\);', 'LOG_SITE;', restored, flags=re.S
    )
    # d9c2bd00 whole GEM source, except the orphan branch and log statements.
    assert (
        hashlib.sha256(normal.encode()).hexdigest()
        == '6270e2fe90a57e478c01fc3ae4dccfa0cce3ee60631193e7c81b42fa48268ee9'
    )
    assert (
        hashlib.sha256(header.encode()).hexdigest()
        == 'beb78a7e0d2ad2f60e42509e2279f8eaf9fa7326003f19e3633c785e3198933d'
    )
    assert 'LOG_ERR(' not in source
    names = [
        'eth_xlnx_gem_configure_buffers',
        'eth_xlnx_gem_handle_rx_pending',
        'eth_xlnx_gem_handle_tx_done',
        'eth_xlnx_gem_send',
        'eth_xlnx_gem_isr',
    ]
    functions = '\n\n'.join(function(source, n) for n in names)
    host = (HERE / 'gem_host.c').read_text().replace('/* MACROS */', macros(header))
    host = host.replace('/* FUNCTIONS */', functions)
    compile_run(host, 'gem_minimal', [1, 13], companions=[BUILD / 'phy_burst.c'])
    reverse = mutation(
        host, orphan_new, orphan_old.replace('LOG_ERR(', 'LOG_ERR_RATELIMIT_RATE(1000, ')
    )
    compile_run(
        reverse, 'gem_reverse_orphan', [1], reverse=True, companions=[BUILD / 'phy_burst.c']
    )
    compile_run(reverse, 'gem_baseline_control', [13], companions=[BUILD / 'phy_burst.c'])
    # No extra RX behavior is admitted to the minimal driver, even if its
    # synthetic throughput test passes.
    changed = mutation(source, 'while (1) {', 'while (false) {')
    changed = mutation(changed, orphan_new, orphan_old)
    changed = re.sub(
        r'LOG_(?:ERR|WRN)(?:_RATELIMIT_RATE)?\(.*?\);', 'LOG_SITE;', changed, flags=re.S
    )
    assert (
        hashlib.sha256(changed.encode()).hexdigest()
        != '6270e2fe90a57e478c01fc3ae4dccfa0cce3ee60631193e7c81b42fa48268ee9'
    )
    print('GEM baseline source equality and reverse: PASS', flush=True)


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
    body = 'struct xlnx_gem_mdio_data { struct k_mutex lock; };\n' + macros(source) + '\n' + body
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
    companion = host[: host.index('int main(int argc, char **argv)')]
    companion += (HERE / 'phy_burst.inc').read_text()
    (BUILD / 'phy_burst.c').write_text(companion)
    # Restore the exact 23fe8c0d layout: recovery fields precede the MMIO slots.
    fields = '\tDEVICE_MMIO_NAMED_RAM(mac);\n\tDEVICE_MMIO_NAMED_RAM(clkc);\n'
    changed = mutation(host, fields, '')
    changed = mutation(
        changed, '\tuint8_t\t\t\t\tmac_addr[6];', '\tuint8_t\t\t\t\tmac_addr[6];\n' + fields
    )
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
    bringup()
    gem()
    phy()
    mdio()
    phy_io()
    coap_log()
