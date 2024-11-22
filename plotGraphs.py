import matplotlib.pyplot as plt

fileNumber = input("Digite a velocidade: ")

fileName = f'logFile-{fileNumber}rads.txt'

data = []
with open(fileName, 'r') as file:
    for line in file:
        print(line.strip())
        data.append(float(line.strip()))

print(data)