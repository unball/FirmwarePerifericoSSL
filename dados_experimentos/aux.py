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
        # ia.append(float(partes[2]))
        # ib.append(float(partes[3]))
        iq.append(float(partes[2]))
        id.append(float(partes[3]))
        # ialpha.append(float(partes[6]))
        # ibeta.append(float(partes[7]))
        # mag.append(float(partes[8]))
        encoder.append(float(partes[4]))
        vel.append(float(partes[5]))


    plt.figure()
    plt.plot(iq, color='blue')
    plt.plot(id, color='red')
    plt.title("iq vs id")
    plt.grid()

    plt.figure()
    plt.plot(entrada, color='blue')
    # plt.plot(mag, color='red')
    plt.title("entrada vs mag")
    plt.grid()


    plt.figure()
    plt.plot(vel)
    plt.title("vel")
    plt.grid()

    plt.show()


entrada = '23-10.txt'

filtrar_tensao_amostra(entrada)