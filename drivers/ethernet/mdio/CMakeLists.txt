# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_MDIO_XLNX_GEM mdio_xlnx_gem.c)
zephyr_library_sources_ifdef(CONFIG_MDIO_XLNX_ESC mdio_xlnx_esc.c)
