print ("c'est parti pour l'aventure")
print (" \n Envoie l'angle horaire chef ! : ")
x = 15*float(input("Ah = "))
print ("\n Maintenant la déclinaison chef ! : ")
z = float(input("dec = "))
print (" \n Pour finir il nous faut la latitude de l'observateur chef ! : ")
y = float(input("lat = "))


O = cos(radians(x)) #cosinus de l'angle horaire
U = sin(radians(x)) #sinus de l'angle horaire
H = cos(radians(z)) #cosinus de la déclinaison
W = sin(radians(z)) #sinus de la déclinaison
N = cos(radians(y)) #cosinus de la latitude de l'observateur
G = sin(radians(y)) #sinus de la latitude de l'observateur

t= N*O*H + W*G
float (t)
print ("\n sin t = ",t,"\n")
a = asin (t)

print (" la hauteur (t) en radians vaut : ",a)
d = degrees (a)
print ("la hauteur en degré vaut",d,"\n")

from math import *
AZIMUT = atan2(H*U, G*H*O-N*W)
AZIMUT= degrees(AZIMUT)+180
print (AZIMUT)
