Import("env")

#
# Dump build environment (for debug)
# print(env.Dump())
#

env.Append(
  LINKFLAGS=[
    "-mfloat-abi=hard",
    "-mfpu=fpv4-sp-d16",
    "-T./firmware/STM32G431CBUx_FLASH.ld",
  ]
)