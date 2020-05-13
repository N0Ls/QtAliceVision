#include "MSfMData.hpp"

#include <QThreadPool>
#include <QFileInfo>
#include <QDebug>

#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

namespace qtAliceVision {

/**
 * @brief QRunnable object dedicated to load sfmData using AliceVision.
 */
class SfmDataIORunnable : public QObject, public QRunnable
{
    Q_OBJECT

public:

    explicit SfmDataIORunnable(const QUrl& sfmDataPath):
    _sfmDataPath(sfmDataPath)
    {}

    /// Load SfM based on input parameters
    Q_SLOT void run() override;

    /**
     * @brief  Emitted when sfmData have been loaded and sfmData objects created.
     */
    Q_SIGNAL void resultReady(aliceVision::sfmData::SfMData*);

    aliceVision::sfmData::SfMData* getSfmData() { return _sfmData; }

private:
    const QUrl _sfmDataPath;
    aliceVision::sfmData::SfMData* _sfmData = nullptr;
};

void SfmDataIORunnable::run()
{
    using namespace aliceVision;

    _sfmData = new sfmData::SfMData();
    try
    {
        if(!sfmDataIO::Load(*_sfmData, _sfmDataPath.toLocalFile().toStdString(), sfmDataIO::ESfMData::ALL))
        {
            qDebug() << "[QtAliceVision] Failed to load sfmData: " << _sfmDataPath << ".";
        }
    }
    catch(std::exception& e)
    {
        qDebug() << "[QtAliceVision] Failed to load sfmData: " << _sfmDataPath << "."
                 << "\n" << e.what();
    }

    Q_EMIT resultReady(_sfmData);
}

void MSfMData::load()
{
    if(_sfmDataPath.isEmpty())
    {
        setStatus(None);
        return;
    }
    if(!QFileInfo::exists(_sfmDataPath.toLocalFile()))
    {
        setStatus(Error);
        return;
    }

    setStatus(Loading);    

    // load features from file in a seperate thread
    SfmDataIORunnable* ioRunnable = new SfmDataIORunnable(_sfmDataPath);
    connect(ioRunnable, &SfmDataIORunnable::resultReady, this, &MSfMData::onSfmDataReady);
    QThreadPool::globalInstance()->start(ioRunnable);
}

void MSfMData::onSfmDataReady(aliceVision::sfmData::SfMData* newSfmData)
{
    _sfmData.reset(newSfmData);
    setStatus(Ready);
}

}

#include "MSfMData.moc"
