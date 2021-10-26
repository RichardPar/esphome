import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import (
    CONF_ID,
    CONF_INVERTED,
    CONF_ICON,
    ENTITY_CATEGORY_CONFIG,
    ICON_POWER,
)

shutdown_ns = cg.esphome_ns.namespace("shutdown")
ShutdownSwitch = shutdown_ns.class_("ShutdownSwitch", switch.Switch, cg.Component)

CONFIG_SCHEMA = (
    switch.SWITCH_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(ShutdownSwitch),
            cv.Optional(CONF_INVERTED): cv.invalid(
                "Shutdown switches do not support inverted mode!"
            ),
            cv.Optional(CONF_ICON, default=ICON_POWER): switch.icon,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(cv.entity_category_schema(default=ENTITY_CATEGORY_CONFIG, config=True))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await switch.register_switch(var, config)
