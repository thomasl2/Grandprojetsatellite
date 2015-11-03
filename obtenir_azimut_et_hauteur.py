from math import *
def spher_to_cart(r,theta,phi):
    """
converti coordonnees spheriques en cartesiens, les angles sont en degres
    """
    x=r*sin(radians(theta))*cos(radians(phi))
    y=r*sin(radians(theta))*sin(radians(phi))
    z=r*cos(radians(theta))
    #return(x,y,z)
    return {'x':x,'y':y,'z':z}

def cart_to_spher(x,y,z):
    """
converti coordonnees cartesiens en spheriques, les angles retournes sont en degres
    """
    r=sqrt(x**2+y**2+z**2)
    phi=atan(y/x)
    theta=atan(sqrt(x**2+y**2)/z)
    return{'r':r,'phi':degrees(phi),'theta':degrees(theta)}


def sub_vect(v,w):
    """
soustrait le vecteur w a v, ce sont des dictionnaires contenant x,y,z
    """
    return {'x':v['x']-w['x'],'y':v['y']-w['y'],'z':v['z']-w['z']}

    

    
r_s=float(input("entrez la l'altitude du satellite "))
theta_s=float(input("entrez la longitude du satellite "))
phi_s=float(input("entrez la lattitude du satellite "))
r_o=float(input("entrez la l'altitude de l'observateur "))
theta_o=float(input("entrez la longitude de l'observateur "))
phi_o=float(input("entrez la lattitude de l'observateur "))


coord_sat = spher_to_cart(r_s, theta_s, phi_s)


coord_obs = spher_to_cart(r_o, theta_o, phi_o)

coord_relat_sat = sub_vect(coord_sat, coord_obs)

#cart_to_spher(coord_relat_sat['x'], coord_relat_sat['y'], coord_relat_sat['z'])
coord_celestes= cart_to_spher(**coord_relat_sat)  #c'est un raccourci, ca disloque le contenu du dictionnaire

print ("les coordonnees celestes sont :", coord_celestes) #c'est pas fini, afficher un beau resultat intermediaire
print("Lest coord celestes sont : Déclinaison:{} asdasd:{}".format(coord_celestes['theta'],coord_celestes['phi']))

def equations_celestes(lat_obs, phi, theta):
    phi = radians(phi)
    theta = radians(theta)
    lat_obs = radians(lat_obs) 
    h = asin(cos(lat_obs))*cos(phi)*cos(theta)+sin(lat_obs)*sin(phi)
    z = acos((sin(lat_obs)*cos(phi)*cos(theta)-cos(lat_obs)*sin(phi))/cosh(h))
    cos_h= cos(phi)*sin(theta)/sin(z)
    sin_z= cos(phi)* sin(theta)/ cos(h)
    if cos_h<0:
        h=-h+pi
    if sin_z < 0:
        z=-z
    return {'h':degrees(h), 'z':degrees(z)}


azimut_et_hauteur = equations_celestes(phi_o, coord_celestes['phi'], coord_celestes['theta'])

print ( azimut_et_hauteur )
    

#delta: phi(lat) Ah:theta(long)


