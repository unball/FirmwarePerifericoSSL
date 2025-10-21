import matplotlib.pyplot as plt

def filtrar_tensao_amostra(arquivo_entrada):
    with open(arquivo_entrada, 'r', encoding='utf-8') as f:
        linhas = f.readlines()

    t0 = []
    entrada = []
    ia = []
    ib = []
    ialpha = []
    ibeta = []
    iq = []
    id = []
    mag = []
    encoder = []
    vel = []

    for i in range(0, len(linhas)-1):
        partes = linhas[i].strip().split()
        t0.append(float(partes[0]))
        entrada.append(float(partes[1]))
        ia.append(float(partes[2]))
        ib.append(float(partes[3]))
        iq.append(float(partes[4]))
        id.append(float(partes[5]))
        ialpha.append(float(partes[6]))
        ibeta.append(float(partes[7]))
        mag.append(float(partes[8]))
        encoder.append(float(partes[9]))
        vel.append(float(partes[10]))



    plt.figure()
    plt.plot(iq, color='blue')
    plt.plot(id, color='red')
    plt.title("iq id")
    # plt.xlim([600,800])
    # plt.title("ia ib")

    plt.figure()
    plt.plot(mag, color='green')
    plt.title("mag")
    # plt.xlim([600,800])

    plt.figure()
    plt.plot(entrada)
    plt.title("entrada")

    plt.figure()
    plt.plot(vel)
    plt.title("vel")
    plt.grid()
    # plt.plot(vel, color='red')
    # plt.plot(ialpha)
    # plt.plot(ibeta)
    # plt.plot(mag, color='green')
    # plt.title("entrada mag")
    # plt.xlim([600,800])

    plt.show()

    # sum = 0
    # for i in range(0,len(vel)):
    #     vel[i] = (30/3.1415)*vel[i]
    #     sum += vel[i]
    
    # kv_media = sum/len(vel)
    # print(f'KV experimental: {kv_media}')
    # print(f'+70% KV: {kv_media+(kv_media*0.7)}')

    

entrada = '20-10-teste-vel-4.txt'

filtrar_tensao_amostra(entrada)