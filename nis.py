import matplotlib.pyplot as plt



f = open("output.log")

l =[]
r =[]

lines = f.readlines()
f.close()

for s in lines:

      if 'NIS' in s:
              s = s.strip().split(':')
              print(s)
              num = float(s[1])
              if 'Lidar' in s[0]:
                   l.append(num)
              elif 'Radar' in s[0]:
                   r.append(num)


print(l)
print(r)


plt.subplot(121)
plt.title('Radar NIS')
plt.plot(r)

plt.subplot(122)
plt.title('Lidar NIS')
plt.plot(l)

plt.show()

