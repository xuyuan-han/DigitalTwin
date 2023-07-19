
import os.path

import opcua

from opcua.server.standard_address_space.standard_address_space_part3 import create_standard_address_space_Part3
from opcua.server.standard_address_space.standard_address_space_part4 import create_standard_address_space_Part4
from opcua.server.standard_address_space.standard_address_space_part5 import create_standard_address_space_Part5
from opcua.server.standard_address_space.standard_address_space_part8 import create_standard_address_space_Part8
from opcua.server.standard_address_space.standard_address_space_part9 import create_standard_address_space_Part9
from opcua.server.standard_address_space.standard_address_space_part10 import create_standard_address_space_Part10
from opcua.server.standard_address_space.standard_address_space_part11 import create_standard_address_space_Part11
from opcua.server.standard_address_space.standard_address_space_part13 import create_standard_address_space_Part13


def fill_address_space(nodeservice):
    create_standard_address_space_Part3(nodeservice)
    create_standard_address_space_Part4(nodeservice)
    create_standard_address_space_Part5(nodeservice)
    create_standard_address_space_Part8(nodeservice)
    create_standard_address_space_Part9(nodeservice)
    create_standard_address_space_Part10(nodeservice)
    create_standard_address_space_Part11(nodeservice)
    create_standard_address_space_Part13(nodeservice)
