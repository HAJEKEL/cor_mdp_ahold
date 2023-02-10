uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8
uint8 STRING  = 9
uint8 VECTOR  = 10
uint8 POSE    = 11
uint8 BOOL    = 12

string name      # Name of field
uint32 offset    # Offset from start of attribute struct
uint8  datatype  # Datatype enumeration, see above
uint32 length    # Length of the datatype (most useful for strings, etc.)
