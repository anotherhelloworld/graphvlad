fout = open('vlad-2009.dot', 'w')
fin = open('vlad-2009.dat', 'r')

fout.write('graph vl-grand {\n')

for line in fin:
    s = line.split()
    u = int(s[0])
    v = int(s[1])
    w = float(s[2])
    fout.write('\t{} -- {} [label = "{}"]\n'.format(v, u, w))

fout.write('}\n')
