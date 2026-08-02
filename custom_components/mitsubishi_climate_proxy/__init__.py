import logging
from homeassistant.core import HomeAssistant
from homeassistant.config_entries import ConfigEntry
from homeassistant.helpers.typing import ConfigType

_LOGGER = logging.getLogger(__name__)

DOMAIN = "mitsubishi_climate_proxy"

async def async_setup(hass: HomeAssistant, config: ConfigType) -> bool:
    """Set up the Mitsubishi Hybrid Climate component via YAML (legacy)."""
    # Return true to allow platform setup (which is handled in climate.py)
    # This allows keeping the old configuration method working if needed,
    # or we can fully deprecate it. For now, we return True.
    return True

async def async_setup_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Set up Mitsubishi Hybrid Climate from a config entry."""
    hass.data.setdefault(DOMAIN, {})

    # Reload the entry when its options change (e.g. toggling coordinator single-target
    # mode via the Options flow) so the new presentation takes effect live without a
    # delete/recreate — keeping the HomeKit accessory ID stable (no re-pair).
    entry.async_on_unload(entry.add_update_listener(_async_update_listener))

    # Forward the setup to the climate platform
    await hass.config_entries.async_forward_entry_setups(entry, ["climate"])

    return True


async def _async_update_listener(hass: HomeAssistant, entry: ConfigEntry) -> None:
    """Reload the config entry when its options are updated."""
    await hass.config_entries.async_reload(entry.entry_id)

async def async_unload_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Unload a config entry."""
    return await hass.config_entries.async_unload_platforms(entry, ["climate"])
