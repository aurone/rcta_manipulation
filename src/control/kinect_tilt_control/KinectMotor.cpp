#include "KinectMotor.h"

KinectMotor::KinectMotor()
{
    m_isOpen = false;
}

KinectMotor::~KinectMotor()
{
    Close();
}

bool KinectMotor::Open()
{
    const XnUSBConnectionString *paths;
    XnUInt32 count;
    XnStatus res;

    // Init OpenNI USB
    res = xnUSBInit();
    if (res != XN_STATUS_OK) {
        xnPrintError(res, "xnUSBInit failed");
        return false;
    }

    // Open "Kinect motor" USB device
    res = xnUSBEnumerateDevices(0x045E /* VendorID */, 0x02B0 /*ProductID */, &paths, &count);
    if (res != XN_STATUS_OK) {
        xnPrintError(res, "xnUSBEnumerateDevices failed");
        return false;
    }

    // Open first found device
    res = xnUSBOpenDeviceByPath(paths[0], &m_dev);
    if (res != XN_STATUS_OK) {
        xnPrintError(res, "xnUSBOpenDeviceByPath failed");
        return false;
    }

    XnUChar buf[1]; // output buffer

    // Init motor
    // res = xnUSBSendControl(m_dev, (XnUSBControlType) 0xc0, 0x10, 0x00, 0x00, buf, sizeof(buf), 0);
    res = xnUSBSendControl(m_dev, XN_USB_CONTROL_TYPE_VENDOR, 0x10, 0x00, 0x00, buf, sizeof(buf), 0);
    if (res != XN_STATUS_OK) {
        xnPrintError(res, "xnUSBSendControl failed");
        Close();
        return false;
    }

    // res = xnUSBSendControl(m_dev, XnUSBControlType::XN_USB_CONTROL_TYPE_VENDOR, 0x06, 0x01, 0x00, NULL, 0, 0);
    res = xnUSBSendControl(m_dev, XN_USB_CONTROL_TYPE_VENDOR, 0x06, 0x01, 0x00, NULL, 0, 0);
    if (res != XN_STATUS_OK) {
        xnPrintError(res, "xnUSBSendControl failed");
        Close();
        return false;
    }
    return true;
}

void KinectMotor::Close()
{
    if (m_isOpen) {
        xnUSBCloseDevice(m_dev);
        m_isOpen = false;
    }
}

bool KinectMotor::Move(int angle)
{
    XnStatus res;

    // Send move control request
    res = xnUSBSendControl(m_dev, XN_USB_CONTROL_TYPE_VENDOR, 0x31, 2 * angle, 0x00, NULL, 0, 0);
    if (res != XN_STATUS_OK) {
        xnPrintError(res, "xnUSBSendControl failed");
        return false;
    }
    return true;
}
