
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
Több különböző szélességű vonallal is készült pálya, amelyek közül a szigetelő szalag szélességével megegyező, kb. 20 mm széles bizonyult a legjobbnak.
A pálya Blenderes környezetére többféle megoldást kipróbáltunk. Legjobbnak az bizonyult, amikor egy az egész zárt görbét tartalmazó "padlódarabon" található a vonal. A másik két verzió a vonalat tratalmazó "aszfaltcsík" és csak magát a vonalat tartalmazó Blender modellek voltak.

A pályákban a követendő vonal három színben váltakozik, kék, sárga és piros. A kék szakasz a közel egyenes és nagy görbületi sugárú vonalból áll, a sárga szakasz több és kissé élesebb kanyarokat tartalmaz, emiatt a robot lasabban is közlekedik rajta. A piros szakasz sok és éles kanyarból áll, ezért a robot itt halad a leglassabban.


A robotnak RViz-ben a bejárt pályát és annak színét is ki kell rajzolnia.


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


<!--## Dependenciák
## Működés -->
## Vizualizáció

#### Mit csinál a neurális hálózat?

A projektben használt **neurális hálózat** egy képosztályozó modell, amely a robot kameraképéből két dolgot ismer fel:

- **Milyen irányba** kell kormányozni a robotot a vonal követéséhez (pl. egyenesen, balra, jobbra, vagy nincs vonal),
- **Milyen színű** vonalat lát a kamera (piros, sárga vagy kék).

A hálózat bemenetére a kamera képe kerül, majd a kimenetén egyszerre becsli az irányt és a vonal színét.  
Az eredmények alapján a robot automatikusan meghatározza, hogyan mozogjon tovább, illetve továbbítja a felismert vonal színét egy külön ROS topikra.

Emellett a kód vizualizálja a hálózat belső rétegeinek úgynevezett **feature map-jeit** is, így betekintést nyerhetsz abba, hogyan dolgozza fel a képet a hálózat.


A neurális háló tanításról készült kép: 

![A neurális hálózat tanítása](https://raw.githubusercontent.com/20vencel03/Vonalkovetes-szinfelismeressel-/main/tb3_project_py/network_model/model_training.png)

#### Neurális hálózat tanításához használt képek

A neurális háló tanításához szükséges képek beszerzése több forrásból történt, melyek kezdetben külön voltak tesztelve, de végül egy kombinált adatbázis került felhasználásra.

- Az első megközelítés az órai tanítókép készítő node átalakítása volt olyan módon, hogy az magától, adott időnként készítsen képet. Ezzel a felvételek úgy készültek, hogy a nem neurális hálón alapuló vonalkövető algoritmus, kicsit hangolt változata mozgatta a robotot, az általunk létrehozott színes pályán. Itt a szortírozás kézzel történt. Természetesen nagyon hasznos továbbfejlesztés lett volna, úgy kialakítani a rögzítő programot, hogy az már szortírozzon is egyben, azonban az volt a tapasztalat, hogy ezzel a konfigurációval a robot túl jól haladt végig a pályán, és így, a készült anyag nem volt kellően változatos.
- Egy másik próbálkozás a kezdetleges hálók felhasználása volt mind mozgatásra és szortírozásra, azonban ez a több hálós módszeren alapul ami elvetésre került, mind erőfrrás igénye, mind pedig nem megbízható működése miatt, így ezen módszerrel végül nem is készült olyan adatbázis ami később felhasználásra került volna.
- Az utolsó módszer pedig a konzultáción javasolt adatbázis sokszorozás volt. Ami azt jelenti, hogy az eredeti órán használt, jól működő háló tanító képei lettek átdolgozva. Egy python kód megkereste és átszinezte a vonalat a képeken. Az így kapott adatbázisból azonban hiányoztak a színátmenetek, ami miatt a belőle tanított háló színérzékenysége nem volt kellően stabil.

Végül az első módszerrel szerzett adatbázisának megválogatott verziója került egyesítésre az utolsó megközelítés adatbázisával, ami már így együtt egy kellően stabil neurális hálót eredményezett



# Eredmények
A projektről futás közben készült videó megtalálható a YouTube-on.

Link: https://youtu.be/CRSvCb_Uejo
