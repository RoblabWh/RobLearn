import matplotlib.pyplot as plt
import numpy as np
import random
from matplotlib.ticker import NullFormatter  # useful for `logit` scale
if __name__ == "__main__":
    lines = [line.rstrip('\n').split(',') for line in open('results.txt')]
    lines2D = [line.rstrip('\n').split(',') for line in open('results2D.txt')]

    x = np.array([])
    y = np.array([])
    z = np.array([])
    avg_x = np.array([])
    avg_y = np.array([])
    avg_z = np.array([])
    i=0
    for time , score , n_steps in lines:
        i += 1
        x = np.append(x, float(score) / float(n_steps))
        y = np.append(y, float(score))
        z = np.append(z, float(n_steps))
        if i % 10 == 0:
            avg_x =np.append(avg_x, np.average(x))
            x = np.array([])
        if i % 100 == 0:
            avg_y = np.append(avg_y, np.average(y))
            avg_z = np.append(avg_z, np.average(z))
            z = np.array([])

            y = np.array([])






    x2D = np.array([])
    y2D = np.array([])
    z2D = np.array([])
    avg_x2D = np.array([])
    avg_y2D = np.array([])
    avg_z2D = np.array([])

    i2 = 0
    for time2D, score2D, n_steps2D in lines2D:

        x2D = np.append(x2D, float(score2D) / float(n_steps2D))
        z2D = np.append(z2D, float(n_steps2D))
        y2D = np.append(y2D, float(score2D))

        if i % 10 == 0:
            avg_x2D =np.append(avg_x2D, np.average(x2D))
            x2D = np.array([])

        if i% 100 ==0:
            avg_y2D = np.append(avg_y2D, np.average(y2D))
            avg_z2D = np.append(avg_z2D, np.average(z2D))
            y2D = np.array([])
            z2D = np.array([])


        i += 1
    plt.figure(1)


    plt.subplot(221)
    plt.plot(avg_x)
    plt.plot(avg_x2D)
    plt.yscale('linear')
    plt.ylabel('Step Score')
    plt.xlabel('Espisodes ( x10 )')
    plt.grid(True)
    plt.title('average Score pro Step')

    plt.subplot(222)

    plt.plot(avg_y)
    plt.plot(avg_y2D)
    plt.yscale('linear')
    plt.ylabel('Score')
    plt.xlabel('Espisodes ( x100 )')
    plt.title('average Score')
    plt.grid(True)


    plt.subplot(223)

    plt.plot(avg_z)
    plt.plot(avg_z2D)
    plt.yscale('linear')
    plt.ylabel('N_Steps')
    plt.xlabel('Espisodes ( x100 )')
    plt.grid(True)
    plt.title('average number of steps')


    plt.subplots_adjust(top=0.92, bottom=0.1, left=0.10, right=0.95, hspace=0.55,
                        wspace=0.35)
    plt.savefig('result')
    plt.show()
