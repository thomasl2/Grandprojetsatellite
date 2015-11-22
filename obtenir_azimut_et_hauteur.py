from math import *

R_TERRE = 6371009

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


def calcul(alt_o, theta_o, phi_o,alt_s, theta_s, phi_s):
    r_o = alt_o + R_TERRE

    r_s = alt_s + R_TERRE

    coord_sat = spher_to_cart(r_s, theta_s, phi_s)

    coord_obs = spher_to_cart(r_o, theta_o, phi_o)

    coord_relat_sat = sub_vect(coord_sat, coord_obs)

    # cart_to_spher(coord_relat_sat['x'], coord_relat_sat['y'], coord_relat_sat['z'])
    coord_celestes = cart_to_spher(**coord_relat_sat)  # c'est un raccourci, ca disloque le contenu du dictionnaire

    print ("les coordonnees celestes sont :", coord_celestes)  # c'est pas fini, afficher un beau resultat intermediaire
    print("Lest coord celestes sont : Déclinaison:{} asdasd:{}".format(coord_celestes['theta'], coord_celestes['phi']))
    '''
    def equations_celestes(lat_obs, phi, theta):
        phi = radians(phi)
        theta = radians(theta)
        lat_obs = radians(lat_obs)
        h = asin(cos(lat_obs))*cos(phi)*cos(theta)+sin(lat_obs)*sin(phi)
        cos_z=(sin(lat_obs)*cos(theta)*cos(phi)-cos(lat_obs)*sin(theta))/cos(h)
        print(cos_z)
        if cos_z>1:
            cos_z=0.99 #eviter erreurs d'arrondis
        print(cos_z)
        z = acos(cos_z)
        cos_h= cos(phi)*sin(theta)/sin(z)
        sin_z= cos(phi)* sin(theta)/ cos(h)
        if cos_h<0:
            h=-h+pi
        if sin_z < 0:
            z=-z
        return {'h':degrees(h), 'z':degrees(z)}
        '''

    # phi=declinaison
    # theta=Angle horaire
    def coord_horaires_vers_horizontales(lat_obs, phi, theta):
        """
        ref. https://www.wikiwand.com/fr/Syst%C3%A8me_de_coordonn%C3%A9es_c%C3%A9lestes#/Des_coordonn.C3.A9es_horaires_aux_coordonn.C3.A9es_horizontales
            Connaissant les valeurs respectives AH et δ de l'angle horaire et de la déclinaison, la hauteur h et l'azimut Z peuvent être obtenus grâce aux trois formules suivantes :
        """
        phi = radians(phi)
        theta = radians(theta)
        lat_obs = radians(lat_obs)
        h = asin(cos(lat_obs) * cos(phi) * cos(theta) + sin(lat_obs) * sin(phi))
        z = atan(cos(phi) * sin(theta) / (sin(lat_obs) * cos(phi) * cos(theta) - cos(lat_obs) * sin(phi)))
        cos_h = cos(phi) * sin(theta) / sin(z)
        cos_z = (sin(lat_obs) * cos(theta) * cos(phi) - cos(lat_obs) * sin(theta)) / cos_h
        if cos_h < 0:
            h = -h + pi
        if cos_z < 0:
            z += pi
        return {'h': degrees(h), 'z': degrees(z)}

    azimut_et_hauteur = coord_horaires_vers_horizontales(phi_o, phi_s, coord_celestes['theta'] - theta_o)

    print(azimut_et_hauteur)
    return(azimut_et_hauteur)


    # delta: phi(lat) Ah:theta(long)

nomfichier=input("Saisisez le nom du fichier csv (ou entree pour un calcul interactif):")
if len(nomfichier)<1:
    alt_s=float(input("entrez la l'altitude du satellite "))
    theta_s=float(input("entrez la longitude du satellite "))
    phi_s=float(input("entrez la lattitude du satellite "))
    alt_o=float(input("entrez la l'altitude de l'observateur "))
    theta_o=float(input("entrez la longitude de l'observateur "))
    phi_o=float(input("entrez la lattitude de l'observateur "))

    calcul(alt_o, theta_o, phi_o,alt_s, theta_s, phi_s)
else:
    import csv
    fichier=open(nomfichier,'r') # ouverture du fichier nomfichier en lecture
    lecteur=csv.DictReader(fichier)
    for l in lecteur:
        alt_o = float(l['alt_o'])
        theta_o = float(l['long_o'])
        phi_o = float(l['lat_o'])
        alt_s = float(l['alt_s'])
        theta_s =float(l['long_s'])
        phi_s = float(l['lat_s'])

        coord_horiz=calcul(alt_o, theta_o, phi_o, alt_s, theta_s, phi_s)


    """
    alternative en utilisant le csv reader simple
    fichier=open(nomfichier,'r',newline='\r') # ouverture du fichier nomfichier en lecture
    lecteur=csv.reader(fichier)
    header=next(lecteur)
    for l in lecteur:
        alt_o = l[2]
        theta_o = l[0]
        phi_o = l[1]
        alt_s = l[5]
        theta_s = l[3]
        phi_s = l[4]
        # on peut fabriquer dictionnaire a partir du header
        # dic=dict(zip(header,l))

        calcul(alt_o, theta_o, phi_o, alt_s, theta_s, phi_s)
    """

# zip(header,l)





