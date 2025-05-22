
# Vonalkövetés színfelismeréssel

Vonalkövetés színfelismeréssel TurtleBot3 robottal és neurális hálóval  
**BME Mechatronika MSc – Kognitív Robotika tárgy projekt feladata**

---

## A megoldandó feladat leírása

A neurális háló nemcsak a vonal irányát, hanem annak színét is meghatározza.  
A robot felismeri a különböző színű vonalakat, a színt kiírja és ennek megfelelően viselkedik.  
A bejárt pályát és a detektált vonalszínt egy külön ROS node vizualizálja az RViz-ben.
A projekt szimulációban fut, de a kód elvileg valós TurtleBot3 robottal is kompatibilis.

---

## A projekt futtatásához szükséges csomagok és könyvtárak

### Rendszerkövetelmények
- Ubuntu 24.04
- ROS 2 [Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- Gazebo (compatible version with Jazzy like Harmonic)

### Ros2 package-k
A Turtlebot3 már nem támogatott, de az alábbi maintainelt package-kkel működik:
```yaml
git clone -b ros2 https://github.com/MOGI-ROS/turtlebot3_msgs
git clone -b mogi-ros2 https://github.com/MOGI-ROS/turtlebot3
git clone -b new_gazebo https://github.com/MOGI-ROS/turtlebot3_simulations
git clone https://github.com/MOGI-ROS/mogi_trajectory_server
```
### Dependenciák
Az alábbi dependenciákat is telepíteni kell:
```yaml
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-dynamixel-sdk
sudo apt install ros-jazzy-rclpy
sudo apt install ros-jazzy-sensor-msgs
sudo apt install ros-jazzy-geometry-msgs
sudo apt install ros-jazzy-std-msgs 
sudo apt install ros-jazzy-cv-bridge
sudo apt install ros-jazzy-ament-index-python
sudo apt install python3-pip
sudo apt install pipx
```


### Python csomagok

A szükséges csomagok egy `requirements.txt` fájlba is kerülhetnek, de manuálisan is telepíthetők:

- `numpy==1.26.4`
- `opencv-python`
-  `tensorflow==2.18.0`
- `imutils`
- `scikit-learn`
- `h5py`
- `torch`
- `torchvision`
- `cv_bridge`
- `matplotlib`
- `colcon-common-extensions` (buildhez szükséges)

#### Telepítés például:
```bash
sudo apt update
sudo apt upgrade
pip install tensorflow==2.18.0
pip install imutils
pip install scikit-learn
pip install opencv-python
pip install matplotlib
pip install numpy==1.26.4
```

## Készített pályák
Több különböző szélességű vonallal is készült pálya, amelyek közül a szigetelő szalag szélességével megegyező, kb. 20 mm széles bizonyult a legjobbnak.
A pálya Blenderes környezetére többféle megoldást kipróbáltunk. Legjobbnak az bizonyult, amikor egy az egész zárt görbét tartalmazó "padlódarabon" található a vonal. A másik két verzió a vonalat tratalmazó "aszfaltcsík" és csak magát a vonalat tartalmazó Blender modellek voltak.

A pályákban a követendő vonal három színben váltakozik, kék, sárga és piros. A kék szakasz a közel egyenes és nagy görbületi sugárú vonalból áll, a sárga szakasz több és kissé élesebb kanyarokat tartalmaz, emiatt a robot lasabban is közlekedik rajta. A piros szakasz sok és éles kanyarból áll, ezért a robot itt halad a leglassabban.


A robotnak RViz-ben a bejárt pályát és annak színét is ki kell rajzolnia.


A pályák megtalálhatók a ./tb3_project/worlds mappában (az sdf fájlok), a .dae fájlok (a Blender pályakészítő program kimenete) pedig a ./gazebo_models mappában találhatók.


## Szimulációs és vizalizációs környezet röviden

### RViz – Vizualizáció a ROS-ban

Az **RViz** egy vizualizációs eszköz a ROS rendszerben, amely lehetővé teszi robotok szenzoradatainak, mozgásának és egyéb állapotainak valós idejű megjelenítését.

Ebben a projektben az RViz az alábbiakat jeleníti meg:

- a TurtleBot3 aktuális helyzetét és mozgását,
- a robot kamerájának képét,
- a neurális hálózat által detektált **vonalszínt és haladási irányt**,
- a robot által megtett útvonalat színezve – a felismert vonal színe alapján.

A színinformációt a rendszer a `line_color` topicon keresztül továbbítja, míg a pozícióadatokat az `odom` topik szolgáltatja.  
Ezek alapján egy külön ROS node (a `Path_marker`) **Marker** típusú objektumokat küld ki a `path_marker` topikra, amelyek a pálya vizuális megjelenítését szolgálják az RViz-ben.

Az RViz konfigurációs fájl a következő helyen található:
./tb3_project/Rviz/turtlebot3_line_follower.rviz

Ez a fájl előre beállított nézeteket, témákat és vizualizációs elemeket tartalmaz, és közvetlenül betölthető az RViz felületén keresztül is.

### Rviz Path_marker node


Egy extra node-ot készítettünk, ami kirajzolja Rvizben a robot által befutott pályát a neurális háló által felismert színnel (a neurális háló be lett tanítva, hogy a kamerakép alapján felismerje a pálya színét). A színt a line_color topicon keresztül kapja meg a node, továbbá a robot helyzetét odometriával, az odom topic-on keresztül kapja meg majd a path_marker topicra kiküld egy Marker típusú objektumot. 
Az RViz minden topikot lát, és a topikok az RViz-ben jelennek meg.


### Gazebo – Szimulációs környezet

A **Gazebo** egy fejlett robotikai szimulációs környezet, amely valósághű fizikai modellezést tesz lehetővé, beleértve robotok mozgását, szenzorait és a környezeti interakciókat.

Ebben a projektben a Gazebo a TurtleBot3 robot mozgásának és érzékelőinek szimulációját végzi. A robot egy egyedi pályán halad, ahol a vonalkövetést és színfelismerést valós időben modellezzük.

A pályák `.sdf` (Simulation Description Format) fájlként találhatók meg a következő könyvtárban:

./tb3_project/worlds/


A Gazebo szimuláció ROS 2-höz kapcsolódik az alábbiakon keresztül:

- A robot állapotát (pl. odometria) és kameraképet ROS 2 topikokon keresztül továbbítja,
- Ezek az adatok közvetlenül elérhetők RViz-ben vagy más ROS node-okban (pl. a neurális hálózat feldolgozó moduljában).

A szimuláció automatikusan elindítja a TurtleBot3-at és betölti a vonalas pályát, amely Blenderből exportált `.dae` modellek segítségével lett létrehozva. Ezek a modellek a következő mappában találhatók:

./gazebo_models/

Ez a felépítés lehetővé teszi, hogy a robot valós hardver nélkül is pontosan tesztelhető és fejleszthető legyen.


# Projekt felépítése

Két főmappából áll a projekt:
*tb3_project
*tb3_project_py
Az előbbi tartalmazza a launch file-t amivel el tudjuk indítani a szimulácit:
```bash
ros2 launch tb3_project simulation.launch
```
A Python package-ben található a path_marker node ami a vizualizációban vesz részt és korábban már bemutatásra került. Továbba a line_follower node is itt található ami a következő paranccsal indítható el:
```bash
ros2 run tb3_project line_follower_cnn
```
vagy
```bash
ros2 run tb3_project line_follower_cnn_robot
```
(Az utóbbinak nincs vizualizációja)

A tb3_project package tartalmazza továbbá mindent ami szükséges a szimulációhoz.
A tb3_project_py package ellenben taralmazza még a neurális hálót, a tanító scriptet és a tanító képeket.

Még egy mappa található a projektben amiben a `.blender` fileok találhatóak.

### Mit csinál a neurális hálózat?

A projektben használt **neurális hálózat** egy képosztályozó modell, amely a robot kameraképéből két dolgot ismer fel:

- **Milyen irányba** kell kormányozni a robotot a vonal követéséhez: egyenesen, balra, jobbra, vagy nincs vonal (forog körbe vonalat keresve),

- **Milyen színű** vonalat lát a kamera (piros, sárga vagy kék).

A hálózat bemenetére a kamera képe kerül, majd a kimenetén egyszerre becsli az irányt és a vonal színét.  
Az eredmények alapján a robot automatikusan meghatározza, hogyan mozogjon tovább, illetve továbbítja a felismert vonal színét egy külön ROS topikra.

A szín hatással van a robot sebességére, kék szín esetén kétszeres, piros esetén félszeres sebességgel megy a sárgához képest.

Emellett a kód vizualizálja a hálózat belső rétegeinek úgynevezett **feature map-jeit** is, így betekintést nyerhetsz abba, hogyan dolgozza fel a képet a hálózat.

A neurális háló tanításról készült kép: 

![A neurális hálózat tanítása](https://raw.githubusercontent.com/20vencel03/Vonalkovetes-szinfelismeressel-/main/tb3_project_py/network_model/model_training.png)

Első ránézésre nagyon rossznak tűnhet, de mivel a tanító képek egyszerre két információt tartalmaznak (irány és szín), de ennek ellenére csak egy címkét adtunk neki, ezért 50% fölé nem nagyon tud elméletileg sem menni.

### Neurális háló tanításához használt képek

A neurális háló tanításához szükséges képek beszerzése több forrásból történt, melyek kezdetben külön voltak tesztelve, de végül egy kombinált adatbázis került felhasználásra.

- Az első megközelítés az órai tanítókép készítő node átalakítása volt olyan módon, hogy az magától, adott időnként készítsen képet. Ezzel a felvételek úgy készültek, hogy a nem neurális hálón alapuló vonalkövető algoritmus, kicsit hangolt változata mozgatta a robotot, az általunk létrehozott színes pályán. Itt a szortírozás kézzel történt. Természetesen nagyon hasznos továbbfejlesztés lett volna, úgy kialakítani a rögzítő programot, hogy az már szortírozzon is egyben, azonban az volt a tapasztalat, hogy ezzel a konfigurációval a robot túl jól haladt végig a pályán, és így, a készült anyag nem volt kellően változatos.
- Egy másik próbálkozás a kezdetleges hálók felhasználása volt mind mozgatásra és szortírozásra, azonban ez a több hálós módszeren alapul ami elvetésre került, mind erőfrrás igénye, mind pedig nem megbízható működése miatt, így ezen módszerrel végül nem is készült olyan adatbázis ami később felhasználásra került volna.
- Az utolsó módszer pedig a konzultáción javasolt adatbázis sokszorozás volt. Ami azt jelenti, hogy az eredeti órán használt, jól működő háló tanító képei lettek átdolgozva. Egy python kód megkereste és átszinezte a vonalat a képeken. Az így kapott adatbázisból azonban hiányoztak a színátmenetek, ami miatt a belőle tanított háló színérzékenysége nem volt kellően stabil.

Végül az első módszerrel szerzett adatbázisának megválogatott verziója került egyesítésre az utolsó megközelítés adatbázisával, ami már így együtt egy kellően stabil neurális hálót eredményezett



# Eredmények
A projektről futás közben készült videó megtalálható a YouTube-on.

Link: https://youtu.be/CRSvCb_Uejo
