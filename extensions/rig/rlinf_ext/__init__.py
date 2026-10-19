# Copyright 2026 The RLinf-Franka Authors.
#
# Out-of-tree RLinf extensions for this rig.

"""Out-of-tree RLinf extensions used by the Franka + Pico collection rig.

RLinf loads this module in every Worker process when ``RLINF_EXT_MODULE`` is
set (see ``load_user_extension_module``), then calls :func:`register` once.
Keeping the rig's vendor glue here means RLinf's own tree stays unmodified.
"""


def register() -> None:
    """Register this rig's camera backends and collection-time behaviours."""
    # Importing the module runs the ``@Camera.register(...)`` decorators.
    from . import cameras  # noqa: F401
    from .collection import (
        install_rich_pico_info,
        register_keyboard_timeout_mode,
    )

    register_keyboard_timeout_mode()
    install_rich_pico_info()
