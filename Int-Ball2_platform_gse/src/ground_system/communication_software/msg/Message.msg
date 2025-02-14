# msg_type
uint8 INTBALL2_SERIALIZED_BINARY_DATA = 0
uint8 INTBALL2_YAML_UTF8 = 1
uint8 DOCK_ROW_BINARY_DATA = 2

uint16 id
uint8 msg_type
string name
uint8[] data
time additional_timestamp