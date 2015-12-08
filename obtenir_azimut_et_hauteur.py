from math import *

R_TERRE = 6371009
#43°36'45.5"N 1°25'44.6"E
phi_o=43.6126389
theta_o= 1.429055555
alt_o=50

def spher_to_cart(r,theta,phi):
    """
converti coordonnees spheriques en cartesiens, les angles sont en degres
    """
    x=r*cos(radians(theta))*cos(radians(phi))
    y=r*cos(radians(phi))*sin(radians(theta))
    z=r*sin(radians(phi))
    return {'x':x,'y':y,'z':z}

def cart_to_spher(x,y,z):
    """
converti coordonnees cartesiens en spheriques, les angles retournes sont en degres
    """
    #x vers meridien de greenwich z pointe vers le pole nord
    r=sqrt(x**2+y**2+z**2)
    phi=asin(z/r)
    r_proj=cos(phi)*r #longeur projection sur le plan xy
    sin_theta= y/r_proj
    theta=acos(x/r_proj)
    if sin_theta<0:
        theta=2*pi-theta
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


    #print("Lest coord celestes sont : Déclinaison:{} asdasd:{}".format(coord_celestes['theta'], coord_celestes['phi']))
   
    def coord_h_vers_hor(y,z,x):
        O = cos(radians(x)) #cosinus de l'angle horaire
        U = sin(radians(x)) #sinus de l'angle horaire
        H = cos(radians(z)) #cosinus de la déclinaison
        W = sin(radians(z)) #sinus de la déclinaison
        N = cos(radians(y)) #cosinus de la latitude de l'observateur
        G = sin(radians(y)) #sinus de la latitude de l'observateur

        t= N*O*H + W*G
        
        #print ("\n sin t = ",t,"\n")
        a = asin (t)

        #print (" la hauteur (t) en radians vaut : ",a)
        h = degrees (a)
        #print ("la hauteur en degré vaut",d,"\n")

        AZIMUT = atan2(H*U, G*H*O-N*W)
        z= degrees(AZIMUT)+180
        
        return {'hauteur': h, 'azimut': z}
    # phi=declinaison
    # theta=Angle horaire
  
    azimut_et_hauteur = coord_h_vers_hor(phi_o, coord_celestes['phi'], theta_o-coord_celestes['theta'])
    #print('az1:',azimut_et_hauteur1)
    #print(azimut_et_hauteur)
    return(azimut_et_hauteur)

nomfichier=input("Saisisez le nom du fichier txt (ou entree pour un calcul interactif):")
if len(nomfichier)<1:
    alt_s=float(input("entrez la l'altitude du satellite "))
    theta_s=float(input("entrez la longitude du satellite "))
    phi_s=float(input("entrez la lattitude du satellite "))
    alt_o=float(input("entrez la l'altitude de l'observateur "))
    theta_o=float(input("entrez la longitude de l'observateur "))
    phi_o=float(input("entrez la lattitude de l'observateur "))

    z=calcul(alt_o, theta_o, phi_o,alt_s, theta_s, phi_s)
    print(z)
else:
    fichier=open(nomfichier,'r') # ouverture du fichier nomfichier en lecture
    ligne=fichier.readline()
    listeligne=[]
    
    while ligne != '' :
        sp=ligne.split('\t')
        listeligne.append(sp)
        ligne=fichier.readline()
    fichier.close()
    print(listeligne)
    
    listeaz=[]
    listehaut=[]
    for col in range (len(listeligne[0])):
        #alt_o = float(l['alt_o'])
        #theta_o = float(l['long_o'])
        #phi_o = float(l['lat_o'])
        alt_s = float(listeligne[2][col])
        theta_s =float(listeligne[0][col])
        phi_s= float(listeligne[1][col])

        coord_horiz=calcul(alt_o, theta_o, phi_o, alt_s, theta_s, phi_s)
        listeaz.append(str(coord_horiz['azimut']))
        listehaut.append(str(coord_horiz['hauteur']))
        print(col,coord_horiz)
        

    fichiersortie=open('azhaut.txt','w')
    fichiersortie.write('\t'.join(listehaut)+'\n')
    fichiersortie.write('\t'.join(listeaz)+'\n')
    fichiersortie.close()


    

        
        
        
    



  







