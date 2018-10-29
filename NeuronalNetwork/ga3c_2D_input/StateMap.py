import numpy as np
#import matplotlib.pyplot as plt
#import random
class StateMap:

    def __init__(self, input_laser):
        self.state = input_laser
        self.height = 100  # self.state.size
        self.witdh = self.state.size
        self.S_map = self.S_map()
        self.S_image = self.creat_image()

    def S_map(self):
        state = np.reshape(self.state, (self.witdh))
        States_map = np.zeros((1, self.height))
        for s_ in state:
            ones = int(s_ * self.height)
            zeros = int(self.height - ones)
            col = np.ones((ones), dtype=float)
            col = np.append(col, np.zeros((zeros), dtype=float))
            col = np.reshape(col, (1, self.height))
            States_map = np.append(States_map, col, axis=0)
        States_map = np.rot90(np.reshape(np.delete(States_map, 0, 0), (self.witdh, self.height)))

        return States_map

    def creat_image(self):

        image_map = np.zeros((1, self.height))
        States_map= np.rot90(self.S_map,1)

        for s in States_map:
            free_count = np.count_nonzero(s)-1

            collision_count = 0

            if (free_count) < 0:
                free_count = 0
                collision_count -= 1

            collision_count += np.count_nonzero(s - 1)


            collisions = np.ones((collision_count), dtype=float) * (-1)
            free = np.ones((free_count), dtype=float) * (1)

            ziel = free_count

            R_col = np.array(np.append(np.append(collisions, ziel), free))


            R_col = np.cumsum(np.flip(R_col * collision_count, 0))  #f,z,c

            R_col = np.flip(self.normalise(R_col), 0) # c,z,f ( c minimum )

            R_col = np.cumsum(R_col*ziel) #c,z,f

            R_col = np.flip(R_col, 0) # f,z,c

            R_col = np.reshape(R_col, (1, self.height))

            image_map = np.append(image_map, R_col, axis=0)
        image_map = np.delete(image_map, 0, 0)
        image_map = np.rot90(image_map, 1)
        image_map = np.reshape(self.normalise(image_map),(self.height,self.witdh))

        #plt.imshow(Reward_map, 'gray')


        #count =random.randrange(1000000,2000000)

        #plt.savefig('rewads/r_%08d' % count)



        return image_map

    def normalise(self, s):
        s = np.array([s])
        shape = s.shape
        max_min = np.amax(s) - np.amin(s)
        min = np.amin(s)
        avg = np.average(s)
        if not max_min == 0:
            s = (s - min) / max_min
        else:
            s = (s - min)

        return np.reshape(s  , shape)

