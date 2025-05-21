*A simulation_bringup_line_follow a saját pályán fusson
*Kipróbál, hogy ezt tudja e követni az eredeti NN
*Az NN változtatása, hogy 4 helyett 7 mappával dolgozzon és két kimenete legyen és tanítás
*Az line_follower_cnn átírása, hogy a frissített NN-t tudja kezelni (első körben 2 szín alapján sebesség nem feltétlen kell)
**vizualizáció átírása ha kell (ezt nem emlékszem most hogy volt)
**viselkedés átírása színek kezelésére
*RVIZ-ben a szín felsmerés alapján színezni a megtett utat.
*A projekt lecsupaszítása -> fölösleges launch fileok és stb töröl (ide elkezdem gyűjteni neveket csak, a végén csináljuk szerintem csak)
**robot_ -os launch fileok 3db
**check_urdf -> nekem az eredeti sem fut
*

#### TODO readme
*projekt dependenciák listázása
*high level működése leírása (rviz, szín alapú sebesség)




# Vonalkövetés színfelismeréssel

Vonalkövetés színfelismeréssel TurtleBot3 robottal és neurális hálóval  
**BME Mechatronika MSc – Kognitív Robotika tárgy**

---

## A megoldandó feladat leírása

A neurális háló nemcsak a vonal irányát, hanem annak színét is meghatározza.  
A robot felismeri a különböző színű vonalakat, a színt kiírja és ennek megfelelően viselkedik.  
A bejárt pályát és a detektált vonalszínt egy külön ROS node vizualizálja RViz-ben.  
A projekt szimulációban fut, de a kód elvileg valós TurtleBot3 robottal is kompatibilis.

---

## A projekt futtatásához szükséges csomagok és könyvtárak

### Rendszerkövetelmények
- Ubuntu 20.04
- ROS 2 Foxy Fitzroy
- Gazebo (a ROS 2 Foxy telepítés része)

### Python csomagok

A szükséges csomagok egy `requirements.txt` fájlba is kerülhetnek, de manuálisan is telepíthetők:

- `numpy`
- `opencv-python`
- `torch`
- `torchvision`
- `cv_bridge`
- `matplotlib`
- `colcon-common-extensions` (buildhez szükséges)

#### Telepítés
```bash
sudo apt update
sudo apt install ros-foxy-cv-bridge python3-colcon-common-extensions
pip3 install numpy opencv-python torch torchvision matplotlib

```




# Vonalkovetes-szinfelismeressel-
Vonalkövetés színfelismeréssel turtlebottal és neurális hálóval / BME Mechatronika MSc Kognitív Robotika tárgya

# A megoldandó feladat leírása
A neurális háló nem csak a vonal irányát, hanem annak színét is meghatározza.
A robot különböző színű vonalakat felismeri, azok színét kiírja.
A bejárt pályát és a vonal színét egy saját ROS node jeleníti meg RViz-ben.
A projekt szimulációban valósul meg, de a kód alkalmas valós roboton (Turtlebot) történő futtatásra is.



## A projekt futtatásához szükséges csomagok és könyvtárak

### Rendszerkövetelmények
- Ubuntu 20.04
- ROS 2 Foxy Fitzroy
- Gazebo (a ROS 2 Foxy telepítés része)

### Python csomagok (általában `requirements.txt`-be is tehetők):
- `numpy`
- `opencv-python`
- `torch`
- `torchvision`
- `cv_bridge`
- `colcon-common-extensions` (buildeléshez)
- `matplotlib` (vizualizációhoz, ha használva van)

Telepítés:
```bash
sudo apt update
sudo apt install ros-foxy-cv-bridge python3-colcon-common-extensions
pip3 install numpy opencv-python torch torchvision matplotlib

## Készített pályák
Különböző színű és szélességű pályákat készítettünk.
A robotnak RViz-ben ki kell rajzolnia a bejárt pályát és a pályaszakaszok színeit.
A pályák eltérnelk szélességükben, van amelyik rendelkezik éllel. A pályák színezése során piros, kék és sárga színeket használtunk.


# Projekt felépítése
## Struktúra
A ./Tracks mappa tartalmazza a pályáka, amiken a robotnak végig kell mennie. 
A ./tb3_project 

## Dependenciák
## Működés
## Vizulizáció

# Neurális Háló



# Eredmények
"videó"
