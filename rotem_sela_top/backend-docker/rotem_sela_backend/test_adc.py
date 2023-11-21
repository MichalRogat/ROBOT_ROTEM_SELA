from Entity import Trailer1, Trailer2, Trailer3, Trailer4, Trailer5
import time
from functions import GenericFunctions
trailer1 = Trailer1(I2CAddress = 0x11)

while True:
    GenericFunctions.callReadNano([trailer1])
