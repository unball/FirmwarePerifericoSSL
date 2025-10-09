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
    plt.plot(ia, color='red')
    plt.plot(ib, color='green')
    # plt.plot(mag, color='cyan')
    # plt.xlim([600,800])
    plt.title("ia ib")

    plt.figure()
    plt.plot(iq, color='red')
    plt.plot(id, color='green')
    plt.title("iq id")
    # plt.xlim([600,800])

    plt.figure()
    # plt.plot(ialpha)
    # plt.plot(ibeta)
    plt.plot(iq, color='green')
    # plt.plot(entrada, color='red')
    plt.title("entrada mag")
    # plt.xlim([600,800])


    plt.figure()
    plt.plot(encoder)
    # plt.plot(ibeta)
    plt.title("encoder")
    # plt.xlim([600,800])

    plt.figure()
    plt.plot(vel)
    # plt.plot(ibeta)
    plt.title("velocidade")
    # plt.xlim([600,800])

    plt.show()

entrada = '8-10_rampa_12V.txt'

filtrar_tensao_amostra(entrada)