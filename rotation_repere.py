#!/usr/bin/python
# -*- coding: utf-8-*-
from __future__ import print_function

from math import *

R_TERRE = 6371009  # en m

def geospher_to_cart(alt,long,lat):
    '''
    converti coordonnees geographiques spheriques en cartesiennes, les angles sont en degres

    :param alt: altitude au dessus niveau de la mer en m
    :param long: longitude en degre [0 - 180] longitude E [-180, 0] longitude W
    :param lat: Latitude en degres  -90 (pole S) 0 (equateur) 90 (pole nord)
    :return: coordonnees geographiques carthesiennes {x: m , y: y, z: z}
    '''

    r = alt + R_TERRE
    long, lat = radians(long), radians(lat)

    x = r * cos(lat) * cos(long)
    y = r * cos(lat) * sin(long)
    z = r * sin(lat)
    return {'x':x,'y':y,'z':z}

def cart_to_geospher(x,y,z):
    """
    converti coordonnees cartesiennes en geographique spheriques, les angles retournes sont en degres
    """
    r = sqrt(x**2 + y**2 + z**2)
    phi = asin(z/r) # donne latitude phi definitive ( intervalle [-pi/2, pi/2])
    r_cos_phi = r * cos(phi)
    if r_cos_phi != 0.0:
        sin_theta = y / r_cos_phi
        theta = acos(x / r_cos_phi) # si sin(theta) > 0 else 2pi - theta (acos intervalle [0, pi]
        if sin_theta < 0:
            theta = 2 * pi - theta
    else:
        theta = 0.0 # quand phi = pi/2 ou -pi/2 (pole nord et sud) la longitude n'a pas de sens (peut etre quelconque)
    return{'alt':r - R_TERRE,'lat':degrees(phi),'long':degrees(theta)}


def spher_to_cart(r,theta,phi):
    """
converti coordonnees spheriques en cartesiens, les angles sont en degres
        en coordonnees geograpiques: r = elevation + rayon moyen terre, theta = longitude (lambda), phi = latitude
    """
    x = r*sin(radians(theta))*cos(radians(phi))
    y = r*sin(radians(theta))*sin(radians(phi))
    z = r*cos(radians(theta))
    #return(x,y,z)
    return {'x':x,'y':y,'z':z}


def cart_to_spher(x,y,z):
    """
converti coordonnees cartesiennes en spheriques, les angles retournes sont en degres
    """
    r=sqrt(x**2+y**2+z**2)
    phi=atan(y/x)
    theta=atan(sqrt(x**2+y**2)/z)

    # return{'r':r,'phi':degrees(phi),'theta':degrees(theta)}
    return {'r': r, 'phi': degrees(phi), 'theta': degrees(theta)}


def sub_vect(v,w):
    """
soustrait le vecteur w a v, ce sont des dictionnaires contenant x,y,z
    """
    return {'x':v['x']-w['x'],'y':v['y']-w['y'],'z':v['z']-w['z']}

def add_vect(v,w):
    """
additionne les vecteurs v et w, ce sont des dictionnaires contenant x,y,z
    retourne las
    """
    return {'x':v['x'] + w['x'],'y':v['y'] + w['y'],'z':v['z'] + w['z']}

def prod_scal(v,w):
    '''
    :param v: vecteur sous forme de dictionnaire {x,y,z}
    :param w: vecteur sous forme de dictionnaire {x,y,z}
    :return: float produit scalaire de v et w
    '''
    return v['x'] * w['x'] + v['y'] * w['y'] + v['z'] * w['z']


def mul_vect(a,v):
    """
    multiplie le vecteur v par le scalaire a
    retourne le resultat sous forme d'un vecteur
    """
    return {'x':a*v['x'], 'y':a*v['y'], 'z':a*v['z']}

def rotation_repere_autour_z(repere, long):
    '''
    :param repere: Dictionnaire contenant les 3 vecteurs (i,j,k) repere orthonorme direct
    :param long: longitude
    :return: (i,j,k) image du repere par la rotation d'angle longitude

     i' = cos(long) i + sin(long) * j
     j' = -sin(long) i + cos(long) * j
     k' = k
    '''
    long = radians(long)
    image_repere = {}

    image_repere['i'] = add_vect(mul_vect(cos(long),repere['i']), mul_vect(sin(long), repere['j']))
    image_repere['j'] = add_vect(mul_vect(-sin(long), repere['i']), mul_vect(cos(long), repere['j']))
    image_repere['k'] = repere['k']
    return image_repere


def rotation_repere_autour_y(repere, lat):
    '''
    :param repere: Dictionnaire contenant les 3 vecteurs (i,j,k) repere orthonorme direct
    :param long: latitude
    :return: (i,j,k) image du repere par la rotation d'angle latitude autour de j

     i' = cos(lat) i + sin(lat) * k
     j' = j
     k' = -sin(long) i + cos(long) * k
    '''
    lat = radians(lat)
    image_repere = {}

    image_repere['i'] = add_vect(mul_vect(cos(lat), repere['i']), mul_vect(sin(lat), repere['k']))
    image_repere['j'] = repere['j']
    image_repere['k'] = add_vect(mul_vect(-sin(lat), repere['i']), mul_vect(cos(lat), repere['k']))
    return image_repere



def temps_sideral_local(longitude):
    # A l'equinoxe le point vernal est a 0h du meridien de Greenwich
    equinoxe2015=datetime.datetime(2015,3,20,22,45) # heure UTC de l'equinoxe de printemps en 2015

    temps_depuis_eqnx = datetime.datetime.utcnow() - equinoxe2015

    # proportion d'annee ecoulee
    temps_sideral_greenwich = (10 + (38*60+29) /3600 + 24.06570982441908 * temps_depuis_eqnx.total_seconds()/(3600*24)) %24
    # une annee point vernal effectue un tour complet
    tsl = (temps_sideral_greenwich + longitude / 15) % 24
    print('based on equinoxe 2015: GMST=%d LMST=%d' %(temps_sideral_greenwich * 15, tsl*15))

    # Counter check with GLST on 2000/01/01
    ndays = (datetime.datetime.utcnow() - datetime.datetime(2000, 1, 1, 0, 0)).total_seconds()/3600/24  # Nombre de jours depuis 1er Janvier 2000
    GMST = (280.46061837 + 360.98564736629 * ndays) % 360
    LMST = (GMST + longitude) % 360
    print('GMST=%d deg LMST(@%d)=%d' % (GMST, longitude, LMST))
    return temps_sideral_greenwich, tsl



def main():

    cart_to_geospher(1, 0, 1)

    alt_s=float(input("altitude du satellite ?"))
    theta_s=float(input("longitude du satellite ?"))
    phi_s=float(input("latitude du satellite ?"))
    alt_o=float(input("altitude de l'observateur ?"))
    theta_o=float(input("longitude de l'observateur ?"))
    phi_o=float(input("latitude de l'observateur ?"))

    r_s = alt_s + R_TERRE
    r_o = alt_o + R_TERRE

    coord_sat = spher_to_cart(r_s, theta_s, phi_s)
    coord_obs = spher_to_cart(r_o, theta_o, phi_o)
    coord_relat_sat = sub_vect(coord_sat, coord_obs)

    coordgeo_sat = geospher_to_cart(alt_s, theta_s, phi_s)
    print('coordgeo_sat=', coordgeo_sat)
    print('verification calcul inverse:', cart_to_geospher(**coordgeo_sat))

    coordgeo_obs = geospher_to_cart(alt_o, theta_o, phi_o)
    print('coordgeo_obs=', coordgeo_obs)
    print('verification calcul inverse:', cart_to_geospher(**coordgeo_obs))
    coordgeocart_obs_sat = sub_vect(coordgeo_sat, coordgeo_obs)
    coordgeospher_obs_sat = cart_to_geospher(**coordgeocart_obs_sat)
    print('coordgeospher_obs_sat = ', coordgeospher_obs_sat, '\n')


    #cart_to_spher(coord_relat_sat['x'], coord_relat_sat['y'], coord_relat_sat['z'])
    coord_celestes = cart_to_spher(**coord_relat_sat)  # ** est un raccourci par rapport a l'appel commente ci-dessus, ca disloque le contenu du dictionnaire


    print("Les coord spheriques locales relatives a l'observateur sont:\n  r={} theta={} phi={}".
          format(coord_celestes['r'], coord_celestes['theta'],coord_celestes['phi']))
    print("Les coord spheriques2 locales relatives a l'observateur sont:\n  alt={} long={} lat={}".
          format(coordgeospher_obs_sat['alt'], coordgeospher_obs_sat['long'], coordgeospher_obs_sat['lat']))

    ''' Repere geographique d'origine centre de la terre
      avec:
       i => pointant vers l'intersection de l'equateur et du meridien de Greenwhich (latitude et longitude 0)
       j => dans le plan de l'equateur pointant vers 90E
       k => pointant vers le pole nord
    '''

    repere_geographique = {'i': {'x': 1, 'y': 0, 'z': 0},
                    'j': {'x': 0, 'y': 1, 'z': 0},
                    'k': {'x': 0, 'y': 0, 'z': 1}
                    }
    repere_geographique1 = rotation_repere_autour_z(repere_geographique, theta_o)
    repere_geographique2 = rotation_repere_autour_y(repere_geographique1, phi_o)
    '''
    si on centre repere_geographique2 sur l'observateur, on a:
     i pointe vers le zenith de l'observateur
     j et k sont dans le plan horizontal de l'observateur avec:
        j pointant vers l'est
        k vers le nord
    '''
    # on determine les coordonnees du vecteur OS dans le  repere apres rotations (repere_geographique2)
    # par projection sur les 3 axes de celui ci
    coordhoriz_cart_obs_sat = {'x': prod_scal(coordgeocart_obs_sat,repere_geographique2['i']),
                               'y': prod_scal(coordgeocart_obs_sat,repere_geographique2['j']),
                               'z': prod_scal(coordgeocart_obs_sat,repere_geographique2['k'])
                               }
    distance_obs_sat = sqrt(prod_scal(coordgeocart_obs_sat, coordgeocart_obs_sat))
    # en utilisant les nouvelles coordonnes on obtient facilement les angles recherches

    if (coordhoriz_cart_obs_sat['x']> distance_obs_sat) : # pour eviter des erreurs lie aux arondis
        distance_obs_sat = coordhoriz_cart_obs_sat['x']
    hauteur = 90 - degrees(acos(coordhoriz_cart_obs_sat['x'] / distance_obs_sat))

    #ici on se place directement en 2d dans le plan des y et z (donc x=0) d'ou la norme calculee avec 2 valeurs uniquement

    azimut_nord_trigo = degrees(acos(coordhoriz_cart_obs_sat['z'] / sqrt(coordgeocart_obs_sat['y']**2 + coordgeocart_obs_sat['z']**2)))
    # comme acos nous donne uniquement des angles  dans [0,180]
    # on doit en fonction de la composante y on prend l'angle symetrique par rapport a z
    if (coordhoriz_cart_obs_sat['y'] > 0): # le satellite est a l'est de l'observateur
        azimut_nord_trigo = 360 - azimut_nord_trigo

    # l'azimut en geographie est mesure en degre retrograde par rapport au nord
    azimut_nord = 360 - azimut_nord_trigo

    # l'azimut en astronomie est mesure en degre retrograde par rapport au sud
    azimut = (180 + azimut_nord) % 360


    print("/n coordonnees horizontales: azimut nord={} azimut sud ={} hauteur:{}".format(azimut_nord, azimut, hauteur))

    #delta: phi(lat) Ah:theta(long)


"""
    def equations_celestes(lat_obs, phi, theta):
        phi = radians(phi)
        theta = radians(theta)
        lat_obs = radians(lat_obs)
        h = asin(cos(lat_obs))*cos(phi)*cos(theta)+sin(lat_obs)*sin(phi)
        z = acos((sin(lat_obs)*cos(phi)*cos(theta)-cos(lat_obs)*sin(phi))/cosh(h))
        cos_h= cos(phi)*sin(theta)/sin(z)
        sin_z= cos(phi)* sin(theta)/ cos(h)
        if cos_h < 0:
            h = -h + pi
            if sin_z < 0:
                z=-z

        return {'h':degrees(h), 'az':degrees(z)}


    azimut_et_hauteur = equations_celestes(phi_o, coord_celestes['phi'], coord_celestes['theta'])
    azimut_et_hauteur = equations_celestes(phi_o, coordgeospher_obs_sat['lat'], coordgeospher_obs_sat['long'])
    print("/n coordonnees horizontales methode 2: ", azimut_et_hauteur)

"""

if __name__ == "__main__":
    main()


