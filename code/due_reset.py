import lgpio
import time

RESET = 23 

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, RESET)
lgpio.gpio_write(h, RESET, 1)
time.sleep(3)
lgpio.gpio_write(h, RESET, 0)
print("Setting reset LOW")
time.sleep(4)
print("setting reset high again")
lgpio.gpio_write(h, RESET, 1)
