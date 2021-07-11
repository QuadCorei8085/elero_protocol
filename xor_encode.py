import sys

# nibble encoding table (replacing value from 0-15 to random values between 0-15
flash_table_encode = [0x08, 0x02, 0x0d, 0x01, 0x0f, 0x0e, 0x07, 0x05, 0x09, 0x0c, 0x00, 0x0a, 0x03, 0x04, 0x0b, 0x06];
 
# helper to print out a message in hex format
def print_msg(msg):
    for i in range(0, len(msg)):
        print('0x{:02X} '.format(msg[i]), end='');
    print('');
    
# bit counting for parity
def count_bits(byte):
    ones = 0;
    mask = 1;
    
    for i in range(0, 8):
        if mask & byte:
            ones += 1;
        mask <<= 1;
        
    return ones & 0x01;

# parity calculation
def calc_parity(msg):
    p = 0;
    for i in range(0, len(msg), 2):
    
        a = count_bits( msg[0 + i] );
        b = count_bits( msg[1 + i] );
        
        #print(a,b);
        
        p |= a ^ b;
        p <<= 1;
        
    msg[7] = ((p << 3)) & 0xFF;
    
# add a value to each nibble in payload between [start; start+len]
def add_r20_to_nibbles(msg, r20, start, length):
    for i in range(start, length):
        d = msg[i];
        
        ln = (d - r20) & 0x0F;
        hn = ((d & 0xF0) - (r20 & 0xF0)) & 0xFF;
        
        msg[i] = hn | ln;
        
        r20 = (r20 - 0x22) & 0xFF;
        
# add values payload bytes
def add_r20_to_nibbles(msg, r20, start, length):
    for i in range(start, length):
        d = msg[i];
        
        ln = (d + r20) & 0x0F;
        hn = ((d & 0xF0) + (r20 & 0xF0)) & 0xFF;
        
        msg[i] = hn | ln;
        
        r20 = (r20 - 0x22) & 0xFF;
        
# xor bytes in an array with 2 values
def xor_2byte_in_array(msg, xor_b0, xor_b1):
    for i in range(2, len(msg), 2):
        msg[i + 0] = msg[i + 0] ^ xor_b0;
        msg[i + 1] = msg[i + 1] ^ xor_b1;

# encode the nibbles using the table
def encode_nibbles(msg):
    for i in range(0, len(msg)):
        nh = (msg[i] >> 4) & 0x0F;
        nl = msg[i] & 0x0F;
        
        dh = flash_table_encode[nh];
        dl = flash_table_encode[nl];
        
        msg[i] = ((dh << 4) & 0xFF) | ((dl) & 0xFF);
        
# encode a message
def encode_msg(msg):
    xor_val0 = msg[0];
    xor_val1 = msg[1];
    
    calc_parity(msg);
    add_r20_to_nibbles(msg, 0xFE, 0, 8);
    xor_2byte_in_array(msg, xor_val0, xor_val1);
    encode_nibbles(msg);

if __name__ == "__main__":
    #index = int(sys.argv[1]);
    
    index = 0x33; # example index in a msg

    # calculate magic number used for encoding (initial and xor values)
    num = (0x00 - (index * 0x708F)) & 0xFFFF;

    
    #input_arr = [(num&0xFF00)>>8, num&0xFF, 0, 0, 0, 0, 0, 0] # example for BUTTON_UP release (0x00 -> no data)
    input_arr = [(num&0xFF00)>>8, num&0xFF, 0x10, 0, 0, 0, 0, 0] # example for BUTTON_UP push (0x10)

    msg = input_arr;        
    print_msg(msg);

    calc_parity(msg);

    print_msg(msg);

    add_r20_to_nibbles(msg, 0xFE, 0, 8);

    print_msg(msg);

    xor_2byte_in_array(msg, (num&0xFF00)>>8, num&0xFF);

    print_msg(msg);

    encode_nibbles(msg);

    print_msg(msg);