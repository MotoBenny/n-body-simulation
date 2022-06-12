#
#Test
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import random
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

G = 6.6743*10**-11  # m3 kg-1 s-2
# Define what a Frame is
Frame = 1  #seconds

#Create the planets
class body:

    def __init__(self,mass,radius,pos,vel):
        self.m = mass
        self.r = radius
        self.x = pos #needs to be a numpy array
        self.v = vel #needs to be a numpy array
        self.F = np.array([0,0,0]) #hoping this is actually a vector

    #Defines a function that will update the position and velocity of the planet based on the total force on that planet
    def update(self):
        accel = self.F/self.m
        t = np.arange(0, Frame*2, Frame)

        #I might not need an ODE to solve this, since I am doing this one step at a time
        def motion(x, t, params):
            # x = (S, V) as vectors
            a = params
            # convert the 6 element vector to 2 3 element vectors of displacement and velocity
            # to use vector formulation of the math
            s, v = x.reshape(2, 3)
            acceleration = np.array([v * t, a])
            # reshape the two vector results back into one for odeint
            return np.reshape(acceleration, 6)

        SV0 = np.array([self.x, self.v])
        # pass reshaped displacement and velocity vector to odeint
        soln = odeint(motion, np.reshape(SV0, 6), t, args=(accel,))
        # Unsure if this will work as I think it will
        self.x,self.v = soln[1].reshape(2,3)

#Calculates the force between two planets
def Force(planet1,planet2):
    dis = planet1.r-planet2.r
    if dis == 0:
        F = np.array([0,0,0])
    else:
        F = G*planet2.m*planet1.m/dis
    return F

# Calculates the total force on all planets from all other planets
def Total_Force(p1, allp):
    other_planets = allp #Makes a list of all the planets but this planet
    force_on_planet = np.array([0,0,0]) #initializes the force on the planet
    for other in other_planets: #loops through the list of the other planets and calculates the force component from all those planets on this planet
        force_on_planet = force_on_planet + Force(p1,other)
    p1.F = force_on_planet #Stores the total force on this planet in the class attributes

#Main Function


    #Finds the forces on all the planets
def updated(planets):
    for p in planets:
        Total_Force(p,planets)
        #updates the position of all the planets. Note we need to find the force on all the planets before moving to the next step of calculating the position updates for all the planets
    for p in planets:
        p.update()


n =  3 #number of planets

#Creates all planets
planets  = [body(random.randint(3, 9),random.randint(3, 9),np.random.rand(3),np.random.rand(3)) for x in range(n)]

# Create the data set
data = []
for i in range(n):
    data.append(planets[i].x)

def updatedata(data,planets):
    updated(planets)
    newdata =[]
    for i in range(n):
        newdata.append(planets[i].x)
    return newdata


# Attach 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)

# Data


# Set the axes properties
ax.set_xlim3d([-2.0, 2.0])
ax.set_xlabel('X')

ax.set_ylim3d([-2.0, 2.0])
ax.set_ylabel('Y')

ax.set_zlim3d([-2.0, 2.0])
ax.set_zlabel('Z')

ax.set_title('3D Test')

# Creating the Animation object
ani = animation.FuncAnimation(fig, updatedata, data, fargs=(planets), interval=50, blit=False)
plt.show()





