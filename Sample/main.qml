import QtQuick 2.0
import QtMultimedia 5.4
import QtQuick.Window 2.0
import QtQuick.Controls 1.2
import QtQuick.Dialogs 1.2
import ARCameraQml 1.0

Rectangle {
    color: "black"

    Camera {
        id: camera
        captureMode: Camera.CaptureVideo
        videoRecorder {
            resolution: "640x480"
            frameRate: 30
        }
    }

    ARCameraItem {
        id: arCameraItem

        mediaObject: camera
        anchors.fill: parent

        Text {
            id: textStatus
            x: parent.x + parent.width - width
            y: 0
            text: arCameraItem.textStatus
            color: Qt.rgba(255, 255, 255, 255)
        }

        Text {
            id: textState
            anchors.left: parent
            color: Qt.rgba(255, 255, 255, 255)
        }

        function updateInterface() {
            loading.enabled = loading.visible = false;
            switch (arCameraState) {
            case ARCameraItem.NotTracking:
                button.enabled = true;
                button.gradient = gradientNotTracking
                buttonText.text = "Start";
                textState.text = "No tracking.";
                break;
            case ARCameraItem.TrackingBegin:
                button.enabled = true;
                button.gradient = gradientTrackingBegin
                buttonText.text = "Force";
                textState.text = "Initialize tracking.";
                break;
            case ARCameraItem.TrackingNow:
                if (arCameraItem.haveReconstruct()) {
                    button.enabled = true;
                    button.gradient = gradientTrackingNow;
                    buttonText.text = "Start reconstruction";
                } else {
                    button.enabled = false;
                }
                textState.text = "Tracking.";
                break;
            case ARCameraItem.LostTracking:
                button.enabled = true;
                button.gradient = gradientLostTracking;
                buttonText.text = "Reset";
                textState.text = "Tracking is lost.";
                break;
            case ARCameraItem.Reconstruction3D:
                button.enabled = true;
                button.gradient = gradientReconstruction;
                buttonText.text = "Process reconstruction";
                textState.text = "Reconstruction.";
                break;
            case ARCameraItem.LostReconstruction3D:
                button.enabled = true;
                button.gradient = gradientLostReconstruction;
                buttonText.text = "Reset";
                textState.text = "Tracking is lost.";
                break;
            default:
                button.enabled = false;
                buttonText.text = "";
                textState.text = "";
            }
            button.visible = button.enabled;
        }

        onArCameraStateChanged: updateInterface()
        Component.onCompleted:  updateInterface()

        onStartRecontruct: {
            loading.enabled = loading.visible = true;
            button.enabled = button.visible = false;
        }
        onEndReconstruct: {
            fileDialog.visible = true;
            fileDialog.folder = "file:/" + arCameraItem.starndartWritableDir();
            //addAction(ARCameraItem.SaveEntity);
            updateInterface();
        }
    }

    Gradient {
        id: gradientNotTracking
        GradientStop { position: 0.0; color: Qt.rgba(0, 0.2, 0.4, 1) }
        GradientStop { position: 0.65; color: Qt.rgba(0, 0.4, 0.8, 1) }
        GradientStop { position: 1.0; color: Qt.rgba(0.8, 0.9, 1.0, 1) }
    }

    Gradient {
        id: gradientTrackingBegin
        GradientStop { position: 0.0; color: Qt.rgba(0, 0.1, 0.2, 1) }
        GradientStop { position: 0.7; color: Qt.rgba(0, 0.2, 0.4, 1) }
        GradientStop { position: 1.0; color: Qt.rgba(0.8, 0.9, 1.0, 1) }
    }

    Gradient {
        id: gradientLostTracking
        GradientStop { position: 0.0; color: Qt.rgba(0.4, 0.2, 0, 1) }
        GradientStop { position: 0.8; color: Qt.rgba(0.8, 0.4, 0, 1) }
        GradientStop { position: 1.0; color: Qt.rgba(1.0, 0.9, 0.8, 1) }
    }

    Gradient {
        id: gradientTrackingNow
        GradientStop { position: 0.0; color: Qt.rgba(0, 0.0, 0.4, 1) }
        GradientStop { position: 0.65; color: Qt.rgba(0, 0.2, 0.8, 1) }
        GradientStop { position: 1.0; color: Qt.rgba(0.8, 0.9, 1.0, 1) }
    }

    Gradient {
        id: gradientReconstruction
        GradientStop { position: 0.0; color: Qt.rgba(0.4, 0.0, 0, 1) }
        GradientStop { position: 0.8; color: Qt.rgba(0.8, 0.2, 0, 1) }
        GradientStop { position: 1.0; color: Qt.rgba(1.0, 0.9, 0.8, 1) }
    }

    Gradient {
        id: gradientLostReconstruction
        GradientStop { position: 0.0; color: Qt.rgba(0.4, 0.0, 0, 1) }
        GradientStop { position: 0.8; color: Qt.rgba(0.8, 0.2, 0, 1) }
        GradientStop { position: 1.0; color: Qt.rgba(1.0, 0.9, 0.8, 1) }
    }

    Rectangle {
        id: button

        radius: 10
        border.color: Qt.rgba(0, 0, 0, 255)
        border.width: 2

        Text {
            id: buttonText
            anchors.centerIn: parent
            color: Qt.rgba(255, 255, 255, 255)
            scale: 2
        }

        width: Math.max(parent.width * 0.35, buttonText.width * buttonText.scale + 20);
        height: Math.max(buttonText.height + 10, 60)

        x: parent.width * 0.5 - width * 0.5
        y: parent.height * 0.75

        MouseArea {
            anchors.fill: parent

            onPressed: button.border.width = 5;
            onReleased: button.border.width = 2;
            onClicked: arCameraItem.addAction(ARCameraItem.NextState);
        }

    }

    Rectangle {
        id: loading

        color: Qt.rgba(0, 0.3, 0.1, 1)
        radius: 20
        border.color: Qt.rgba(0, 0, 0, 1)
        border.width: 3

        x: 0
        y: parent.height - height

        width: parent.width
        height: 50

        Rectangle {
            x: 3
            y: 3

            radius: 20 - 3 * 2

            color: Qt.rgba(0.3, 1, 0.6, 1)

            height: 50 - 3 * 2
            width: parent.width * arCameraItem.procentReconstruction;
        }
    }

    FileDialog {
        id: fileDialog
        title: "Please choose a dir"
        selectMultiple: false
        selectExisting: true
        selectFolder: true

        onAccepted: {
            console.log("You chose: " + fileUrl.toString());
            arCameraItem.addAction(ARCameraItem.SaveEntity, fileUrl.toString());
        }
        onRejected: {
            console.log("Canceled")
        }
    }

}
