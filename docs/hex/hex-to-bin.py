import sys


def to_bin(chars):
    scale = 16  # equals to hexadecimal
    num_of_bits = 8
    return bin(int(chars, scale))[2:].zfill(num_of_bits)


input = sys.argv[1]
if not input:
    exit(1)

f = open(input, "r")
hexfile = f.read()
binfile = ""
n = 2
char_pairs = [hexfile[i:i+n] for i in range(0, len(hexfile), n)]

for hexbyte in char_pairs:
    s = "\n\n"
    if hexbyte != "\n\n":
        in_binary = to_bin(hexbyte)
        s = "%s\n%s\n" % (hexbyte, in_binary)
    binfile += s

out = open("%s.bin" % input, "w")
out.write(binfile)
