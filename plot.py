import matplotlib.pyplot as plt

file = open('Test.txt','r')
data = file.readlines()

# print(float(data[5].split(' ')[0]))
file.close()
# exit()

x = list(range(500))
y = []
for num in range(500):
    y.append(float(data[num].split(' ')[0]))
    print(data[num])

plt.plot(x,y)
plt.show()