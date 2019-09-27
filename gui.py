from PyQt5.QtCore import Qt, QRect, QThread, pyqtSignal
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QIcon, QPixmap, QRegion, QImage
from PyQt5.QtPrintSupport import QPrinter, QPrintDialog
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPushButton, QLineEdit, QLabel,
                             QGroupBox, QMenu, QAction, QTextEdit, QMessageBox,
                             QFileDialog)

import cv2
import numpy as np
import serial
import socket
import os

class gpsThread(QThread):
    gpsSignal = pyqtSignal(object)

    def __init__(self):
        QThread.__init__(self)

    def gps(self):
        global decData

        while True:
            # Look for the response
            bufsize = 2048
            data = sock.recv(bufsize)
            decData = data.decode('utf-8')

            parsed_data = list(self.parse(decData))

            self.gpsSignal.emit(parsed_data)

    def parse(self, decData):
        decDataList = decData.split(";")

        # cLat
        currentLatGpsVal = decDataList[0]
        cLat = currentLatGpsVal[4:]

        # cLon
        currentLonGpsVal = decDataList[1]
        cLon = currentLonGpsVal[4:]

        return cLat, cLon

    def run(self):
        self.gps()
        self.exec()


class Window(QMainWindow):
    global adrString
    adrString = serial.Serial("COM3", 9600)

    def __init__(self):
        super().__init__()

        self.coord = gpsThread()

        getPrimaryScreenSettings = app.primaryScreen()
        sizeScreen = getPrimaryScreenSettings.size()
        self.originalWidth = sizeScreen.width()
        self.originalHeight = sizeScreen.height()
        sizeScreen = getPrimaryScreenSettings.availableGeometry()
        self.screenWidth = sizeScreen.width()
        self.screenHeight = sizeScreen.height()

        self.circleSize = self.screenWidth * 0.008
        self.blackFrameWidth = self.screenWidth * 0.125
        self.blackFrameHeight = self.screenHeight * 0.075

        self.flyLight = QColor(Qt.white)
        self.homeLight = QColor(Qt.white)

        #get the height of the taskbar
        self.heightDiff = self.originalHeight - self.screenHeight

        #(starting x positon, starting y postion, width, height)
        self.setGeometry(0, self.heightDiff, self.screenWidth, self.screenHeight - self.heightDiff)

        self.title = 'Autonomous Target Scoring Drone - Kevin Le, Morgan Lytle, & Alec Pflaumer'
        self.initUI()

    #MAIN PAGE
    def initUI(self):
        self.setWindowTitle(self.title)

###########################################################################
############################## D I S P L A Y ##############################
        global sock

        # Create a TCP/IP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect the socket to the port where the server is listening
        server_address = ('192.168.1.10', 50007)
        print('connecting to %s port %s' % server_address)
        sock.connect(server_address)

        # Create home input boarder
        homeBoarder = QGroupBox(self)
        homeBoarder.setTitle("Set Home Coordinate")
        homeBoarder.move(self.screenWidth * 0.03, self.screenHeight * 0.82)
        homeBoarder.resize(self.screenWidth * 0.2725, self.screenHeight * 0.078)

        global homeTextBoxLat
        # Latitude Home Text Box
        inputHomeLabelLat = QLabel(self)
        inputHomeLabelLat.setText("Latitude:")
        inputHomeLabelLat.move(self.screenWidth * 0.037, self.screenHeight * 0.846)
        homeTextBoxLat = QLineEdit(self)
        homeTextBoxLat.move(self.screenWidth * 0.065, self.screenHeight * 0.85)
        homeTextBoxLat.resize((self.screenWidth * 0.8817) - (self.screenWidth * 0.825), self.screenHeight * 0.0265)

        global homeTextBoxLon
        # Longitude Home Text Box
        inputhomeLabelLon = QLabel(self)
        inputhomeLabelLon.setText("Longitude:")
        inputhomeLabelLon.move(self.screenWidth * 0.137, self.screenHeight * 0.846)
        homeTextBoxLon = QLineEdit(self)
        homeTextBoxLon.move(self.screenWidth * 0.17, self.screenHeight * 0.85)
        homeTextBoxLon.resize((self.screenWidth * 0.8817) - (self.screenWidth * 0.825), self.screenHeight * 0.0265)

        #Create input text box boarder
        textBoarder = QGroupBox(self)
        textBoarder.setTitle("Input GPS Coordinate")
        textBoarder.move(self.screenWidth * 0.34, self.screenHeight * 0.82)
        textBoarder.resize(self.screenWidth*0.2725, self.screenHeight * 0.115)

        global textBoxLat
        #Latitude Text Box
        inputLabelLat = QLabel(self)
        inputLabelLat.setText("Latitude:")
        inputLabelLat.move(self.screenWidth * 0.347, self.screenHeight * 0.846)
        textBoxLat = QLineEdit(self)
        textBoxLat.move(self.screenWidth * 0.375, self.screenHeight * 0.85)
        textBoxLat.resize((self.screenWidth*0.8817)-(self.screenWidth*0.825), self.screenHeight * 0.0265)

        global textBoxLon
        #Longitude Text Box
        inputLabelLon = QLabel(self)
        inputLabelLon.setText("Longitude:")
        inputLabelLon.move(self.screenWidth * 0.447, self.screenHeight * 0.846)
        textBoxLon = QLineEdit(self)
        textBoxLon.move(self.screenWidth * 0.48, self.screenHeight * 0.85)
        textBoxLon.resize((self.screenWidth*0.8817)-(self.screenWidth*0.825), self.screenHeight * 0.0265)

        ################################################
        #####Current GPS for Latitude and Longitude#####
        ################################################
        currentGpsBox = QGroupBox(self)
        currentGpsBox.setTitle("Current GPS Coordinate")
        currentGpsBox.move(self.screenWidth * 0.795, self.screenHeight * 0.07)
        currentGpsBox.resize((self.screenWidth*0.8817)-(self.screenWidth*0.72), self.screenHeight * 0.095)

        #Current Latitude
        global currGpsLat
        currentLabelLat = QLabel(self)
        currentLabelLat.setText("Latitude:")
        currentLabelLat.move(self.screenWidth * 0.802, self.screenHeight * 0.0905)
        #NEED TO TRY TO MAKE THIS WORK WITH ALEC'S GPS CODE
        currGpsLat = QLineEdit(self)
        currGpsLat.setReadOnly(True)
        currGpsLat.move(self.screenWidth * 0.835, self.screenHeight * 0.0905)
        currGpsLat.resize(((self.screenWidth * 0.8817) - (self.screenWidth * 0.77)), self.screenHeight * 0.026)

        #Current Longitude
        global currGpsLon
        currentLabelLon = QLabel(self)
        currentLabelLon.setText("Longitude:")
        currentLabelLon.move(self.screenWidth * 0.8017, self.screenHeight * 0.125)
        currGpsLon = QLineEdit(self)
        currGpsLon.setReadOnly(True)
        currGpsLon.move(self.screenWidth * 0.835, self.screenHeight * 0.125)
        currGpsLon.resize(((self.screenWidth * 0.8817) - (self.screenWidth * 0.77)), self.screenHeight * 0.026)
        ################################################
        #####Desired GPS for Latitude and Longitude#####
        ################################################
        desireGpsBox = QGroupBox(self)
        desireGpsBox.setTitle("Desired GPS Coordinate")
        desireGpsBox.move(self.screenWidth * 0.795, self.screenHeight * 0.18)
        desireGpsBox.resize((self.screenWidth * 0.8817) - (self.screenWidth * 0.72), self.screenHeight * 0.095)

        #Desire Latitude
        global desGpsLat
        desireLabelLat = QLabel(self)
        desireLabelLat.setText("Latitude:")
        desireLabelLat.move(self.screenWidth * 0.802, self.screenHeight * 0.2005)
        desGpsLat = QLineEdit(self)
        desGpsLat.setReadOnly(True)
        desGpsLat.move(self.screenWidth * 0.835, self.screenHeight * 0.2005)
        desGpsLat.resize(((self.screenWidth * 0.8817) - (self.screenWidth * 0.77)), self.screenHeight * 0.026)

        #Desire Longitude
        global desGpsLon
        desireLabelLon = QLabel(self)
        desireLabelLon.setText("Longitude:")
        desireLabelLon.move(self.screenWidth * 0.802, self.screenHeight * 0.235)
        desGpsLon = QLineEdit(self)
        desGpsLon.setReadOnly(True)
        desGpsLon.move(self.screenWidth * 0.835, self.screenHeight * 0.235)
        desGpsLon.resize(((self.screenWidth * 0.8817) - (self.screenWidth * 0.77)), self.screenHeight * 0.026)

        ########################################################
        #####Home GPS Coordinate for Latitude and Longitude#####
        ########################################################
        homeGpsBox = QGroupBox(self)
        homeGpsBox.setTitle("Home GPS Coordinate")
        homeGpsBox.move(self.screenWidth * 0.795, self.screenHeight * 0.29)
        homeGpsBox.resize((self.screenWidth * 0.8817) - (self.screenWidth * 0.72), self.screenHeight * 0.095)

        # Desire Latitude
        global homeGpsLat
        homeLabelLat = QLabel(self)
        homeLabelLat.setText("Latitude:")
        homeLabelLat.move(self.screenWidth * 0.802, self.screenHeight * 0.3105)
        homeGpsLat = QLineEdit(self)
        homeGpsLat.setReadOnly(True)
        homeGpsLat.move(self.screenWidth * 0.835, self.screenHeight * 0.3105)
        homeGpsLat.resize(((self.screenWidth * 0.8817) - (self.screenWidth * 0.77)), self.screenHeight * 0.026)

        # Desire Longitude
        global homeGpsLon
        homeLabelLon = QLabel(self)
        homeLabelLon.setText("Longitude:")
        homeLabelLon.move(self.screenWidth * 0.802, self.screenHeight * 0.345)
        homeGpsLon = QLineEdit(self)
        homeGpsLon.setReadOnly(True)
        homeGpsLon.move(self.screenWidth * 0.835, self.screenHeight * 0.345)
        homeGpsLon.resize(((self.screenWidth * 0.8817) - (self.screenWidth * 0.77)), self.screenHeight * 0.026)

        ############################
        #####User Score Boarder#####
        ############################
        userScoreBox = QGroupBox(self)
        userScoreBox.setTitle("Score")
        userScoreBox.move(self.screenWidth * 0.795, self.screenHeight * 0.4)
        userScoreBox.resize((self.screenWidth * 0.8817) - (self.screenWidth * 0.7165), self.screenHeight * 0.199)

        #FIX USER SCORE VALUE!!!!
        global score
        totalScoreLabel = QLabel(self)
        totalScoreLabel.setText("Total Score:")
        totalScoreLabel.move(self.screenWidth * 0.8015, self.screenHeight * 0.42)
        score = QLineEdit(self)
        score.setReadOnly(True)
        score.move(self.screenWidth * 0.84, self.screenHeight * 0.42)
        score.resize(((self.screenWidth * 0.8817) - (self.screenWidth * 0.77)), self.screenHeight * 0.026)

        #Number of Hits
        #FIX NUM HITS VALUE!!!!
        global hits
        totalHitsLabel = QLabel(self)
        totalHitsLabel.setText("Total Hits:")
        totalHitsLabel.move(self.screenWidth * 0.8015, self.screenHeight * 0.4545)
        hits = QLineEdit(self)
        hits.setReadOnly(True)
        hits.move(self.screenWidth * 0.84, self.screenHeight * 0.4545)
        hits.resize(((self.screenWidth * 0.8817) - (self.screenWidth * 0.77)), self.screenHeight * 0.026)

        #100 Points
        #FIX POINTS100 VALUE!!!!
        global p100
        p100Label = QLabel(self)
        p100Label.setText("Hits of 100:")
        p100Label.move(self.screenWidth * 0.8015, self.screenHeight * 0.489)
        p100 = QLineEdit(self)
        p100.setReadOnly(True)
        p100.move(self.screenWidth * 0.84, self.screenHeight * 0.489)
        p100.resize(((self.screenWidth * 0.8817) - (self.screenWidth * 0.77)) * 0.15, self.screenHeight * 0.026)

        #50 Points
        #FIX POINTS50 VALUE!!!!
        global p50
        p50Label = QLabel(self)
        p50Label.setText("Hits of 50:")
        p50Label.move(self.screenWidth * 0.8015, self.screenHeight * 0.5235)
        p50 = QLineEdit(self)
        p50.setReadOnly(True)
        p50.move(self.screenWidth * 0.84, self.screenHeight * 0.5235)
        p50.resize(((self.screenWidth * 0.8817) - (self.screenWidth * 0.77)) * 0.15, self.screenHeight * 0.026)

        #10 Points
        #FIX POINTS10 VALUE!!!!
        global p10
        p10Label = QLabel(self)
        p10Label.setText("Hits of 25:")
        p10Label.move(self.screenWidth * 0.8015, self.screenHeight * 0.558)
        p10 = QLineEdit(self)
        p10.setReadOnly(True)
        p10.move(self.screenWidth * 0.84, self.screenHeight * 0.558)
        p10.resize(((self.screenWidth * 0.8817) - (self.screenWidth * 0.77)) * 0.15, self.screenHeight * 0.026)

        ledBox = QGroupBox(self)
        ledBox.setTitle("Flight Status")
        ledBox.move(self.screenWidth * 0.625, self.screenHeight * 0.065)
        ledBox.resize((self.screenWidth * 0.8817) - (self.screenWidth * 0.819), self.screenHeight * 0.085)

        #LED Fly to Destination
        flyLed = QLabel(self)
        flyLed.setText("Drone in Flght")
        flyLed.move(self.screenWidth * 0.63, self.screenHeight * 0.085)

        #LED Fly to Home
        homeLed = QLabel(self)
        homeLed.setText("Flying Home")
        homeLed.move(self.screenWidth * 0.63, self.screenHeight * 0.117)

        #Picture Boarder
        picBox = QGroupBox(self)
        picBox.setTitle("Image Command")
        picBox.move(self.screenWidth * 0.545, self.screenHeight * 0.675)
        picBox.resize((self.screenWidth * 0.8817) - (self.screenWidth * 0.8225), self.screenHeight * 0.11)

        #Text Editor
        self.text = QTextEdit(self)
        self.text.setPlaceholderText("Enter comments")
        self.text.setGeometry(self.screenWidth * 0.55, self.screenHeight * 0.2, self.screenWidth * 0.23, self.screenHeight * 0.397)

        #UCR Logo
        navy = QLabel(self)
        navyImage = QPixmap("ONR_logo4.png")
        navy.setPixmap(navyImage)
        navy.setGeometry(self.screenWidth * 0.62, self.screenHeight * 0.6, self.screenWidth * 0.6, self.screenHeight * 0.397)

        #Navy Logo
        ucr = QLabel(self)
        ucrImage = QPixmap("UCR_logo2.png")
        ucr.setPixmap(ucrImage)
        ucr.setGeometry(self.screenWidth * 0.8, self.screenHeight * 0.6, self.screenWidth * 0.6, self.screenHeight * 0.397)
############################### D I S P L A Y #############################
###########################################################################

###########################################################################
############################## B U T T O N S ##############################
        setButton = QPushButton('Set', self)
        setButton.resize(setButton.sizeHint())
        setButton.move(self.screenWidth * 0.24, self.screenHeight * 0.85)
        setButton.clicked.connect(self.clickSet)

        self.goButton = QPushButton('Go', self)
        self.goButton.resize(self.goButton.sizeHint())
        self.goButton.move(self.screenWidth * 0.55, self.screenHeight * 0.85)
        self.goButton.clicked.connect(self.clickGo)

        homeButton = QPushButton('Home', self)
        homeButton.resize(homeButton.sizeHint())
        homeButton.move(self.screenWidth * 0.55, self.screenHeight * 0.89)
        homeButton.clicked.connect(self.clickHome)

        picButton = QPushButton('Picture', self)
        picButton.resize(picButton.sizeHint())
        picButton.move(self.screenWidth * 0.55, self.screenHeight * 0.7)
        picButton.clicked.connect(self.clickPicture)

        displayButton = QPushButton('Display', self)
        displayButton.resize(displayButton.sizeHint())
        displayButton.move(self.screenWidth * 0.55, self.screenHeight * 0.745)
        displayButton.clicked.connect(self.clickDisplay)

        saveButton = QPushButton('Save', self)
        saveButton.resize(saveButton.sizeHint())
        saveButton.move(self.screenWidth * 0.55, self.screenHeight * 0.61)
        saveButton.clicked.connect(self.clickSave)

        openButton = QPushButton('Open', self)
        openButton.resize(openButton.sizeHint())
        openButton.move(self.screenWidth * 0.641, self.screenHeight * 0.61)
        openButton.clicked.connect(self.clickOpen)

        clearButton = QPushButton('Clear', self)
        clearButton.resize(clearButton.sizeHint())
        clearButton.move(self.screenWidth * 0.732, self.screenHeight * 0.61)
        clearButton.clicked.connect(self.clickClear)
############################## B U T T O N S ##############################
###########################################################################

###########################################################################
############################## T O O L B A R ##############################
        toolBar = self.menuBar()
        fileMenu = toolBar.addMenu('File')

        #File sub-menus
        #save as...
        save_menu = QMenu('Save', self)
        save_menu_act = QAction('Drone Info.', self)
        save_menu.addAction(save_menu_act)
        save_menu_act.triggered.connect(self.saveStatusMessages)
        save_menu.setIcon(QIcon('save.png'))
        fileMenu.addMenu(save_menu)

        #Print
        print_menu = QMenu("Print", self)
        print_menu_act1 = QAction('GUI test',self) #NEEDS TO BE EDITED LATER IN THE ' '
        print_menu.addAction(print_menu_act1)
        print_menu_act1.triggered.connect(self.screenShot)
        print_menu.setIcon(QIcon('printer.png'))
        fileMenu.addMenu(print_menu)
############################## T O O L B A R ##############################
###########################################################################

###########################################################################
############################## B U T T O N S ##############################
    def currentCoord(self, location):
        currGpsLat.setText(location[0])
        currGpsLon.setText(location[1])

        currentLat = location[0]
        currentLon = location[1]

        adrString.write(("cLat" + str(currentLat) + ",cLon" + str(currentLon) + str(",;")).encode())
        print("cLat" + str(currentLat) + str(",") + "cLon" + str(currentLon) + str(",;"))

        QApplication.processEvents()

    def clickSet(self):
        global inputHomeTextLat
        global inputHomeTextLon
        inputHomeTextLat = homeTextBoxLat.text()
        inputHomeTextLon = homeTextBoxLon.text()

        homeGpsLat.setText(inputHomeTextLat)
        homeGpsLon.setText(inputHomeTextLon)

        self.update()

    def clickGo(self):
        global inputTextBoxLatValue
        global inputTextBoxLonValue

        inputTextBoxLatValue = textBoxLat.text()
        inputTextBoxLonValue = textBoxLon.text()
        message = QMessageBox()
        desReply = message.question(self, 'GPS Confirmation', "Latitude: " + str(inputTextBoxLatValue) + "\nLongitude: " + str(inputTextBoxLonValue), message.Yes | message.No, message.No)

        if(desReply == message.Yes):
            latValue = inputTextBoxLatValue
            lonValue = inputTextBoxLonValue
            desGpsLat.setText(latValue)
            desGpsLon.setText(lonValue)
            #self.update()

            adrString.write(("dLat" + str(latValue) + ",dLon" + str(lonValue) + str(",;")).encode())
            print("DESIRED GPS STRING EXAMPLE:   " + "dLat" + str(latValue) + ",dLon" + str(lonValue) + str(",;"))

            self.coord.start()

            self.coord.gpsSignal.connect(self.currentCoord)

            #Change color of LED
            self.flyLight = QColor(Qt.green)
            self.homeLight = QColor(Qt.white)

            QApplication.processEvents()
            self.update()

    def clickHome(self):
        global inputHomeTextLat
        global inputHomeTextLon
        inputHomeTextLat = homeTextBoxLat.text()
        inputHomeTextLon = homeTextBoxLon.text()

        desGpsLat.setText(inputHomeTextLat)
        desGpsLon.setText(inputHomeTextLon)

        adrString.write(("dLat" + str(inputHomeTextLat) + str(",") + "dLon" + str(inputHomeTextLon) + str(",;")).encode())
        print("HOME GPS STRING EXAMPLE:   " + "dLat" + str(inputHomeTextLat) + str(",") + "dLon" + str(inputHomeTextLon) + str(",;"))

        self.coord.gpsSignal.connect(self.currentCoord)

        #Chaning color of led
        self.homeLight = QColor(Qt.yellow)
        self.flyLight = QColor(Qt.white)

        self.update()

    def clickPicture(self):

        req = 'p'
        reqEn = req.encode('utf-8')
        print('sending "%s"' % req)
        sock.sendall(reqEn)

    def clickDisplay(self):
        #Contains CV code
        image = cv2.imread("target.png")
        resizeImage = cv2.resize(image, (0, 0), fx=0.305, fy=0.305)
        copyResizeImage = resizeImage.copy()

        grayScale = cv2.cvtColor(resizeImage, cv2.COLOR_BGR2GRAY)

        # Otsu Filter
        ret1, otsuThresh = cv2.threshold(grayScale, 155, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Closing Filter - cancels out the noise from the grass area
        kernel = np.ones((6, 6), np.uint8)
        closing = cv2.morphologyEx(otsuThresh, cv2.MORPH_CLOSE, kernel)

        # Opening Filter - fills in the square to be whole
        opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)

        try:
            # Drawing boundary box
            image2, contours, hierarchy = cv2.findContours(opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # With each contour draw boundingRect
            for c in contours:
                # Get the bounding rect
                x2, y2, w, h = cv2.boundingRect(c)
                # Draw a green rectangle to visualize the bounding rect
                cv2.rectangle(copyResizeImage, (x2, y2), (x2 + w, y2 + h), (0, 255, 100), 2)

            # Connected Componenets
            connectComp = 8
            connect = cv2.connectedComponentsWithStats(opening, connectComp, cv2.CV_32S)
            numLabel = connect[0]
            centroid = connect[3]
            num = int(numLabel)

            # -----Detects Target center-----
            heightMid = int(y2 + (h / 2))
            widthMid = int(x2 + (w / 2))

            xyCent = centroid[0]
            xCent = xyCent[0]
            yCent = xyCent[1]
            xDiffTarget = abs(xCent - widthMid)
            yDiffTarget = abs(yCent - heightMid)
            target = 0

            for i in range(1, num):
                xyCent = centroid[i]
                xCent = xyCent[0]
                yCent = xyCent[1]

                xDiff = abs(xCent - widthMid)
                yDiff = abs(yCent - heightMid)

                if ((xDiff <= xDiffTarget) and (yDiff <= yDiffTarget)):
                    xDiffTarget = xDiff
                    yDiffTarget = yDiff
                    target = i

            # Displays the total amount of hits in the picture
            totalHitsTarget = str(target)
            hits.setText(totalHitsTarget)

            targetCentroid = centroid[target]
            # Target tuple used for printing on image
            targetTuple = (int(targetCentroid[0]), int(targetCentroid[1]))
            # -----End target center detection-----

            # Output for Centroid Numbering
            cv2.putText(copyResizeImage, str('x'), targetTuple, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

            # Detecting Circles
            median = cv2.medianBlur(grayScale, 15)

            circles = cv2.HoughCircles(median, cv2.HOUGH_GRADIENT, 0.5, 120, param1=50, param2=30, minRadius=0, maxRadius=150)
            #circles = cv2.HoughCircles(median, cv2.HOUGH_GRADIENT, 1.25, 24, param1=50, param2=30, minRadius=0, maxRadius=150)
            circlesFound = np.uint16(np.around(circles))
            numCircles = np.size(circlesFound, 1)

            count = 1
            for i in circlesFound[0, :]:
                cv2.circle(copyResizeImage, (i[0], i[1]), i[2], (50, 200, 200), 5)
                cv2.circle(copyResizeImage, (i[0], i[1]), 2, (255, 0, 0), 3)
                cv2.putText(copyResizeImage, "Hit " + str(count), (i[0] - 70, i[1] + 30), cv2.FONT_HERSHEY_SIMPLEX, 1.1,
                            (255, 0, 0), 2)
                count = count + 1

            # -------Scoring Algorithm-------
            scores = [0] * numCircles
            totalScore = 0
            total25 = 0
            total50 = 0
            total100 = 0
            averageScore = 0
            # Find corners of regions

            proportion = .33
            # corner point of second region

            w3 = int(w - (w * proportion))
            h3 = int(h - (h * proportion))

            x3 = int(x2 + (w * proportion / 2))
            y3 = int(y2 + (h * proportion / 2))

            # corner point of first region

            w4 = int(w3 - (w3 * (proportion + .08)))
            h4 = int(h3 - (h3 * (proportion + .08)))

            x4 = int(x3 + (w4 * proportion))
            y4 = int(y3 + (h4 * proportion))

            cv2.rectangle(copyResizeImage, (x3, y3), (x3 + w3, y3 + h3), (0, 0, 255), 2)
            cv2.rectangle(copyResizeImage, (x4, y4), (x4 + w4, y4 + h4), (0, 0, 255), 2)

            for i in range(0, numCircles):
                hitX = circlesFound[0, i, 0]
                hitY = circlesFound[0, i, 1]
                # Center region
                if ((hitX > x4) and (hitX < (x4 + w4)) and (hitY > y4) and (hitY < (y4 + h4))):
                    scores[i] = 100
                    total100 += 1
                # Second region
                elif (((hitX > x3) and (hitX < (x3 + w3)) and (hitY > y3) and (hitY < (y3 + h3)))):
                    scores[i] = 50
                    total50 += 1
                elif (((hitX > x2) and (hitX < (x2 + w)) and (hitY > y2) and (hitY < (y2 + h)))):
                    scores[i] = 25
                    total25 += 1
                else:
                    scores[i] = 0
                totalScore += scores[i]

            newTotalScore = str(totalScore)
            score.setText(newTotalScore)

            newTotal100 = str(total100)
            p100.setText(newTotal100)

            newTotal50 = str(total50)
            p50.setText(newTotal50)

            newTotal25 = str(total25)
            p10.setText(newTotal25)

            averageScore = np.mean(scores)

            # Display onto GUI
            height, width, byteValue = copyResizeImage.shape
            byteValue = byteValue * width
            cv2.cvtColor(copyResizeImage, cv2.COLOR_BGR2RGB, image)
            mQImage = QImage(copyResizeImage, width, height, byteValue, QImage.Format_RGB888)

            displayImage = QLabel(self)
            pixmap = QPixmap(mQImage)
            displayImage.setPixmap(pixmap)
            displayImage.setGeometry(self.screenWidth * 0.0135, self.screenHeight * 0.04, self.screenWidth * 0.52, self.screenHeight * 0.75)
            displayImage.show()

        except:
            # Display onto GUI
            height, width, byteValue = resizeImage.shape
            byteValue = byteValue * width
            cv2.cvtColor(resizeImage, cv2.COLOR_BGR2RGB, image)
            mQImage = QImage(resizeImage, width, height, byteValue, QImage.Format_RGB888)

            displayImage = QLabel(self)
            pixmap = QPixmap(mQImage)
            displayImage.setPixmap(pixmap)
            displayImage.setGeometry(self.screenWidth * 0.0135, self.screenHeight * 0.04, self.screenWidth * 0.52, self.screenHeight * 0.75)
            displayImage.show()

    def clickSave(self):
        fileName = QFileDialog.getSaveFileName(self, 'Save File', os.getenv('HOME'))
        with open(fileName[0], 'w') as f:
            myText = self.text.toPlainText()
            f.write(myText)

    def clickOpen(self):
        fileName = QFileDialog.getOpenFileName(self, 'Open File', os.getenv('HOME'))
        with open(fileName[0], 'r') as f:
            fileText = f.read()
            self.text.setText(fileText)

    def clickClear(self):
        self.text.clear()
############################## B U T T O N S ##############################
###########################################################################

###########################################################################
################################ P A I N T ################################

    def paintEvent(self, QPaintEvent):
        global painter
        painter = QPainter(self)
        painter.begin(self)

        # White LED indicating nothing happening
        painter.setPen(QPen(Qt.black, 2, Qt.SolidLine))
        # Drone in Flight LED
        painter.setBrush(QBrush(self.flyLight, Qt.SolidPattern))
        painter.drawEllipse(self.screenWidth * 0.675, self.screenHeight * 0.094, self.circleSize, self.circleSize)
        # Flying Home LED
        painter.setBrush(QBrush(self.homeLight, Qt.SolidPattern))
        painter.drawEllipse(self.screenWidth * 0.675, self.screenHeight * 0.125, self.circleSize, self.circleSize)

        # Boarder for image
        painter.setPen(QPen(Qt.black, 3, Qt.SolidLine))
        # Top line
        painter.drawLine(self.blackFrameWidth * 0.1, self.blackFrameHeight * 0.65, self.blackFrameWidth * 4.265,
                         self.blackFrameHeight * 0.65)
        # Bottom line
        painter.drawLine(self.blackFrameWidth * 0.1, self.blackFrameHeight * 10.4, self.blackFrameWidth * 4.265,
                         self.blackFrameHeight * 10.4)
        # Left line
        painter.drawLine(self.blackFrameWidth * 0.1, self.blackFrameHeight * 0.65, self.blackFrameWidth * 0.1,
                         self.blackFrameHeight * 10.358)
        # Right line
        painter.drawLine(self.blackFrameWidth * 4.265, self.blackFrameHeight * 0.65, self.blackFrameWidth * 4.265,
                         self.blackFrameHeight * 10.358)

        painter.end()

################################ P A I N T ################################
###########################################################################

###########################################################################
############################## T O O L B A R ##############################
    def saveStatusMessages(self):
        #SAVE SUB-MENU
        StatMsgText = StatMsgs.toPlainText()
        StatMsgFile = open('StatusMessagesGui.txt','w')
        StatMsgFile.write(StatMsgText)
        StatMsgFile.close()
        print("Status Message Saved")

    def screenShot(self):
        #PRINT SUB-MENU
        printer = QPrinter(QPrinter.HighResolution)
        dialog = QPrintDialog(printer, self)
        if dialog.exec_() == QPrintDialog.Accepted:
            painter = QPainter()
            painter.begin(printer)
            screen = self.grab()
            painter.setClipRegion(QRegion(QRect(120, 20, 4800, 4500), type = QRegion.Rectangle),
                                      operation = Qt.ReplaceClip)
            pixmap = QPixmap.copy(screen, rect = QRect(120, 20, 1152, 1124))
            pixmap2 = pixmap.scaledToWidth(4400)
            pixmap2 = pixmap.scaledToHeight(4400)
            painter.drawPixmap(0, 0, pixmap2)
            painter.end()
############################## T O O L B A R ##############################
###########################################################################

if __name__ == '__main__':
    import sys

    app = QApplication(sys.argv)  # argv allows to call application by commandLine
    GUI = Window()
    GUI.showMaximized()
    GUI.show()
    sys.exit(app.exec_())
