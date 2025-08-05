

# Test procedure guide (English)

## Requirements

INSTALL PYTHON 3.10.11  (python-3.10.11-amd64.exe for instance)

Please install the necessary libraries with the following commands : 

pip install fpdf2

pip install numpy

pip install serial

pip install pyserial

pip install requests

pip install sounddevice

## Steps

Step 1 : Download "production_test" branch from LYNKX38/LYNKX_firmware and "main" branch from LYNKX32/LYNKX_bootloader on GitHub

Step 2 : Flash new board with "LYNKX_bootloader\Core\Src\main.c"

Step 3 : Prepare your test environment by plugging to your computer : a serial backport cable, a scanning device for QR code and Bar code, the tinySA (spectrum analyzer)

Step 4 : Launch Python script "LYNKX_firmware/Test sources/production_test.py", a Tkinter interface should be running

Step 5 : Prepare a command box by sticking Bar codes and stick QR code on the back of the device

Step 6 : Complete "Operator name" and "COM Port" fields (COM Port number in peripherals configuration panel), scan QR code and Bar code

Step 7 : Click on "Select test firmware" and select ".bin" file, click on "Select firmware" and select ".blf" file

Step 8 : Click on "Configure" and follow Tkinter console instructions

If test procedure is complete, a PDF saving file is created. To proceed to an other test, unplug the current device, click on "Restart" and scan the next device. Then click on "Configure"

Step 9 : (optional) adjust limits for measured values (sound, ble, lora) in functions: verif_power_audio, check_ble_freq, check_lora_freq
# 

# Guide procedure de test (Français)

## Prérequis

Veuillez installer les librairies nécessaires avec les commandes suivantes : 

pip install fpdf2

pip install numpy

pip install serial

pip install pyserial

pip install requests

pip install sounddevice

## Étapes

Étape 1 : Télécharger la branche "production_test" de LYNKX38/LYNKX_firmware et la branche "main" de LYNKX32/LYNKX_bootloader sur GitHub

Étape 2 : Flasher la carte avec "LYNKX_bootloader\Core\Src\main.c"

Étape 3 : Preparer votre environnement de test en branchant à votre ordinateur : un port série compatible avec le port de la balise, une scannette pour QR code et code barre, l'analyseur de spectre tinySA

Étape 4 : Lancer le script Python "LYNKX_firmware/Test sources/production_test.py", une interface Tkinter devrait s'ouvrir

Étape 5 : Preparer un carton commande avec les etiquettes code barre

Étape 6 : Compléter les champs "Operator name" et "COM Port" (Le numéro du port COM est accessible via le gestionnaire de périphériques), scanner le QR code et le code barre

Étape 7 : Cliquer sur "Select test firmware" et selectionner le fichier ".bin", cliquer sur "Select firmware" et selectionner le fichier ".blf"

Étape 8 : Cliquer sur "Configure" et suivre les instructions dans la console Tkinter

Étape 9 (optionnelle) : ajuster les seuils pour les mesures (son, ble, lora) dans les fonctions verif_power_audio, check_ble_freq, check_lora_freq

Si la procédure de test est réussie, un récapitulatif PDF est créé. Pour faire un autre test, débrancher le produit actuel, cliquer sur "Restart" et scanner le prochain produit. Cliquer sur "Configure"
#
