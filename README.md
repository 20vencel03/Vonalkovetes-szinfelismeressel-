
# Vonalkövetés színfelismeréssel

Vonalkövetés színfelismeréssel TurtleBot3 robottal és neurális hálóval  
**BME Mechatronika MSc – Kognitív Robotika tárgy**

---

## A megoldandó feladat leírása

A neurális háló nemcsak a vonal irányát, hanem annak színét is meghatározza.  
A robot felismeri a különböző színű vonalakat, a színt kiírja és ennek megfelelően viselkedik.  
A bejárt pályát és a detektált vonalszínt egy külön ROS node vizualizálja az RViz-ben.
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

## Készített pályák
Különböző színű és szélességű pályákat készítettünk.
A robotnak RViz-ben ki kell rajzolnia a bejárt pályát és a pályaszakaszok színeit.
A pályák eltérnek a szélességükben, van, amelyik rendelkezik éllel.

A pályák színezése során piros, kék és sárga színeket használtunk.

A pályák megtalálhatók a ./tb3_project/worlds mappában (az sdf fájlok), a .dae fájlok (a Blender pályakészítő program kimenete) pedig a ./gazebo_models mappában találhatók.


### RViz röviden

Az **RViz** egy vizualizációs eszköz a ROS rendszerben, amely lehetővé teszi a robotok szenzoradatainak, mozgásának és állapotainak valós idejű megjelenítését.  
Ebben a projektben az RViz segítségével láthatod:

- a TurtleBot3 robot aktuális helyzetét és mozgását a szimulációban,
- a kamera által látott képet,
- a neurális hálózat által detektált vonal színét és irányát.

Az RViz a `/cmd_vel`, `/camera/image_raw` és más ROS topikokból származó adatokat jeleníti meg vizuálisan, így egyszerűen ellenőrizhető, hogy a robot hogyan követi a színes vonalat.

Az RViz beállításai a ./tb3_project/Rviz/turtlebot3_line_follower.rviz fájlban találhatók, és az RViz-ben megnyitva grafikusan szerkeszthetők.FAz 
#### Rviz Path_marker node


Egy extra node-ot készítettünk, ami kirajzolja Rvizben a robot által befutott pályát a neurális háló által felismert színnel (a neurális háló be lett tanítva, hogy a kamerakép alapján felismerje a pálya színét). A színt a line_color topicon keresztül kapja meg a node, továbbá a robot helyzetét odometriával, az odom topic-on keresztül kapja meg majd a path_marker topicra kiküld egy Marker típusú objektumot. 
Az RViz minden topikot lát, és a topikok az RViz-ben jelennek meg.


### Gazebo

A **Gazebo** egy robotikai szimulációs környezet, amely lehetővé teszi különféle robotok, érzékelők és környezetek valósághű modellezését.  
Ebben a projektben a Gazebo biztosítja a TurtleBot3 robot és a pálya szimulációját, így a robot mozgása, szenzorai és a vonalkövetés folyamata anélkül tesztelhető, hogy valódi hardverre lenne szükség.

A Gazebo együttműködik a ROS-szal, így a robot szimulált érzékelőadatai és vezérlése ugyanúgy elérhetők, mintha egy valós roboton dolgoznánk.


# Projekt felépítése


## Dependenciák
## Működés
## Vizualizáció

#### Mit csinál a neurális hálózat?

A projektben használt **neurális hálózat** egy képosztályozó modell, amely a robot kameraképéből két dolgot ismer fel:

- **Milyen irányba** kell kormányozni a robotot a vonal követéséhez (pl. egyenesen, balra, jobbra, vagy nincs vonal),
- **Milyen színű** vonalat lát a kamera (piros, sárga vagy kék).

A hálózat bemenetére a kamera képe kerül, majd a kimenetén egyszerre becsli az irányt és a vonal színét.  
Az eredmények alapján a robot automatikusan meghatározza, hogyan mozogjon tovább, illetve továbbítja a felismert vonal színét egy külön ROS topikra.

Emellett a kód vizualizálja a hálózat belső rétegeinek úgynevezett **feature map-jeit** is, így betekintést nyerhetsz abba, hogyan dolgozza fel a képet a hálózat.




# Eredmények
A projektről futás közben készült videó megtalálható a YouTube-on.

Link: https://youtu.be/CRSvCb_Uejo
