#ifndef GRABBER_H
#define GRABBER_H

#include "QThread"

#include "GrabberLib_global.h"
#include "QConsoleDebugStream.h"

class GRABBERLIBSHARED_EXPORT GrabberRS : public QThread
{
protected:
    std::string m_sIni;
    bool    m_bStopRequested;
    enum GRAB_STATE
    {
        GRABBING = 0,
        STOPPED = 1,
    };

    GRAB_STATE m_GrabState;
    unsigned m_uPayloadSize;

    struct STRUCT_BUFFER_HEADER
    {
        unsigned m_uSlots;          // Number of slots
        unsigned m_uSizeOfFrame;    // Size of a frame
        int m_iLastFrame;      // Last frame
        int m_iLastRead;
        unsigned m_uImageWidth;     // Width of image
        unsigned m_uImageHeight;	// Height
        double    m_RecTick;    // Tickcount of reception time
        unsigned m_uBlockId;		// camera block id
        unsigned m_uSizeOfLastFrame;
    };

    STRUCT_BUFFER_HEADER    m_BufferHeader;
    unsigned char*          m_pBuffer;
    Q_ConsoleDebugStream    logStream;

    QString lastError;

    QString qsStringParameter;
    QString qsStringParameterValue;
    bool bSetStringParameterPending;

    bool bLastTrigger;
    bool bTrigger;
public:


public:
    GrabberRS(std::string sIniFile);
    ~GrabberRS();
    virtual void run();
    void stopGrabbing();
    void startGrabbing();
    unsigned getImageSize();
    unsigned getImageWidth();
    unsigned getImageHeight();
    int getImage(unsigned short *pBuffer, unsigned uBufferSize, unsigned uTimeout);
    QString getLastError()
    {
        return lastError;
    }

    void setCameraParameter(QString, QString);
public slots:
    void setTrigger(bool);

    // new symbols
private:

};



#endif // GRABBER_H
