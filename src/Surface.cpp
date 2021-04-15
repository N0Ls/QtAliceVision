#include "Surface.hpp"

#include <aliceVision/camera/IntrinsicBase.hpp>
#include <aliceVision/camera/Pinhole.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Import M_PI
#include <math.h>
#include <memory>

namespace qtAliceVision {


inline aliceVision::Vec2 toEquirectangular(const aliceVision::Vec3& spherical, int width, int height)
{
    const double vertical_angle = asin(spherical(1));
    const double horizontal_angle = atan2(spherical(0), spherical(2));

    const double latitude = ((vertical_angle + M_PI_2) / M_PI) * height;
    const double longitude = ((horizontal_angle + M_PI) / (2.0 * M_PI)) * width;

    return aliceVision::Vec2(longitude, latitude);
}

Surface::Surface(int subdivisions, QObject* parent)
    : QObject(parent)
{
    updateSubdivisions(subdivisions);
}

Surface::~Surface()
{
}

QPoint Surface::getVertex(int index)
{
    return vertex(index);
}

void Surface::setVertex(int index, float x, float y)
{
    QPoint point(x, y);
    vertex(index) = point;
    Q_EMIT verticesChanged();
}

void Surface::displayGrid(bool display)
{
    setDisplayGrid(display);
    Q_EMIT verticesChanged();
}

void Surface::defaultControlPoints()
{
    clearVertices();
    reinitialize(true);
    Q_EMIT gridChanged();
}

bool Surface::reinit()
{
    return hasReinitialized();
}

void Surface::setupLensDistortion(bool distortion)
{
    if (distortion)
        setViewerType(ViewerType::DISTORTION);
    else
        setViewerType(ViewerType::DEFAULT);

    clearVertices();
    Q_EMIT verticesChanged();
}

void Surface::updateSubdivisions(int subs)
{
    setSubsChanged(true);
    setSubdivisions(subs);

    clearVertices();
    Q_EMIT gridChanged();
}

QPoint Surface::getPrincipalPoint()
{
    return principalPoint();
}

void Surface::setPanoramaViewerEnabled(bool state)
{
    if (state)
        setViewerType(ViewerType::PANORAMA);
    else
        setViewerType(ViewerType::DEFAULT);
}

void Surface::rotatePanoramaRadians(float yawRadians, float pitchRadians)
{
    incrementRotationValues(yawRadians, pitchRadians);

    Q_EMIT verticesChanged();
}

void Surface::rotatePanoramaDegrees(float yawDegrees, float pitchDegrees)
{
    const double yawRadians = yawDegrees * (M_PI / 180.0f);
    const double pitchRadians = pitchDegrees * (M_PI / 180.0f);

    setRotationValues(yawRadians, pitchRadians);

    Q_EMIT verticesChanged();
}

void Surface::mouseOver(bool state)
{
    setMouseOver(state);

    Q_EMIT verticesChanged();
}

// return pitch in degrees
double Surface::getPitchDegree()
{
    // Radians
    const int power = _pitch / M_PI_2;
    double pitch = fmod(_pitch, M_PI_2) * pow(-1, power);

    // Degres
    pitch *= (180.0f / M_PI);
    if (power % 2 != 0) pitch = -90.0 - pitch;

    return pitch;
}

// return yaw in degrees
double Surface::getYawDegree()
{
    // Radians
    int power = _yaw / M_PI;
    double yaw = fmod(_yaw, M_PI) * pow(-1, power);

    yaw *= (180.0f / M_PI);
    if (power % 2 != 0) yaw = -180.0 - yaw;

    return yaw;
}

bool Surface::isMouseInside(float mx, float my)
{
    QPoint P(mx, my);
    bool inside = false;

    for (size_t i = 0; i < indexCount(); i += 3)
    {
        QPoint A = vertex(index(i));
        QPoint B = vertex(index(i + 1));
        QPoint C = vertex(index(i + 2));

        // Compute vectors        
        QPoint v0 = C - A;
        QPoint v1 = B - A;
        QPoint v2 = P - A;

        // Compute dot products
        float dot00 = QPoint::dotProduct(v0, v0);
        float dot01 = QPoint::dotProduct(v0, v1);
        float dot02 = QPoint::dotProduct(v0, v2);
        float dot11 = QPoint::dotProduct(v1, v1);
        float dot12 = QPoint::dotProduct(v1, v2);

        // Compute barycentric coordinates
        float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
        float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        // Check if point is in triangle
        if ((u >= 0) && (v >= 0) && (u + v < 1))
        {
            inside = true;
            break;
        }
    }

    return inside;
}

bool Surface::update(QSGGeometry::TexturedPoint2D* vertices, quint16* indices, QSize textureSize, bool updateSfmData)
{
    // Load Sfm Data File only if needed
    if( (isLensDistortionViewerEnabled() || isPanoramaViewerEnabled()) 
        && (_loadedSfmPath != _sfmPath)
        )
    {
        updateSfmData = loadSfmData();
    }

    // Compute Vertices coordinates and Indices order
    computeGrid(vertices, indices, textureSize, updateSfmData);

    // If Panorama has been rotated, reset values and return true
    if (_isPanoramaRotating)
    {
        _isPanoramaRotating = false;
        return true;
    }

    return updateSfmData;
}

bool Surface::isPanoramaViewerEnabled() const
{
    return _viewerType == ViewerType::PANORAMA;
}

bool Surface::isLensDistortionViewerEnabled() const
{
    return _viewerType == ViewerType::DISTORTION;
}

void Surface::computeGrid(QSGGeometry::TexturedPoint2D* vertices, quint16* indices, QSize textureSize, bool updateSfmData)
{
    // Retrieve intrisics only if sfmData has been updated, and then Compute vertices
    aliceVision::camera::IntrinsicBase* intrinsic = nullptr;
    if (updateSfmData || _isPanoramaRotating)
    {
        std::set<aliceVision::IndexT> intrinsicsIndices = _sfmData.getReconstructedIntrinsics();
        intrinsic = _sfmData.getIntrinsicPtr(*intrinsicsIndices.begin());

        if (intrinsic)
        {
            computePrincipalPoint(intrinsic, textureSize);
            computeVerticesGrid(vertices, textureSize, intrinsic);
        }
    }
    // If there is no sfm data update or intrinsics are invalid, keep the same vertices
    if ((!updateSfmData && !_isPanoramaRotating) || !intrinsic)
    {
        computeVerticesGrid(vertices, textureSize, nullptr);
    }

    // TODO : compute indices only if subs has changed
    computeIndicesGrid(indices);
}

void Surface::computeVerticesGrid(QSGGeometry::TexturedPoint2D* vertices, QSize textureSize, 
    aliceVision::camera::IntrinsicBase* intrinsic)
{
    // Retrieve pose if Panorama Viewer is enable
    aliceVision::sfmData::CameraPose pose;
    if (isPanoramaViewerEnabled() && intrinsic)
    {
        const aliceVision::sfmData::View& view = _sfmData.getView(_idView);
        pose = _sfmData.getPose(view);
        _deletedColIndex.clear();
    }

    const bool fillCoordsSphere = _coordsSphereDefault.empty();
    int vertexIndex = 0;
    int subdivisions = intrinsic ? _subdivisions : 1;
    for (size_t i = 0; i <= subdivisions; ++i)
    {
        for (size_t j = 0; j <= subdivisions; ++j)
        {
            float x = 0.0f;
            float y = 0.0f;
            //if (_vertices.empty())
            {
                x = i * textureSize.width() / (float)subdivisions;
                y = j * textureSize.height() / (float)subdivisions;
            }/*
            else
            {
                x = _vertices[vertexIndex].x();
                y = _vertices[vertexIndex].y();
            }*/

            // TODO : update when subs change
            const float u = i / (float)subdivisions;
            const float v = j / (float)subdivisions;

            // Remove Distortion only if sfmData has been updated
            if (intrinsic)
            {
                // Equirectangular convertion if in panorama context
                if (isPanoramaViewerEnabled())
                {
                    // Compute pixel coordinates on the Unit Sphere
                    if (fillCoordsSphere)
                    {
                        // Image System Coordinates
                        aliceVision::Vec2 uvCoord(x, y);
                        const auto& transfromPose = pose.getTransform();
                        _coordsSphereDefault.push_back(aliceVision::camera::applyIntrinsicExtrinsic(transfromPose, intrinsic, uvCoord));
                    }

                    // Rotate Panorama if some rotation values exist
                    aliceVision::Vec3 coordSphere(_coordsSphereDefault[vertexIndex]);
                    rotatePanorama(coordSphere);

                    // Compute pixel coordinates in the panorama coordinate system
                    aliceVision::Vec2 coordPano = toEquirectangular(coordSphere, _panoramaWidth, _panoramaHeight);

                    /*
                    // If image is on the seem
                    if (vertexIndex > 0)
                    {
                        double deltaX = coordPano.x() - vertices[vertexIndex - 1].x;
                        if (abs(deltaX) > 0.7 * _panoramaWidth)
                        {
                            _deletedColIndex.push_back(std::pair<int, int>(j - 1, i));
                        }
                    }
                    if (vertexIndex >= (_subdivisions + 1))
                    {
                        double deltaY = coordPano.x() - vertices[vertexIndex - (_subdivisions + 1)].x;
                        if (abs(deltaY) > 0.7 * _panoramaWidth)
                        {
                            _deletedColIndex.push_back(std::pair<int, int>(j - 1, i));
                        }
                    }*/
                    vertices[vertexIndex].set(coordPano.x(), coordPano.y(), u, v);
                }
                else
                {
                    const aliceVision::Vec2 undisto_pix(x, y);
                    const aliceVision::Vec2 disto_pix = intrinsic->get_d_pixel(undisto_pix);
                    vertices[vertexIndex].set(disto_pix.x(), disto_pix.y(), u, v);
                }
            }
            else
            {
                vertices[vertexIndex].set(x, y, u, v);
            }
            ++vertexIndex;
        }
    }
}

void Surface::computeIndicesGrid(quint16* indices)
{
    int index = 0;
    for (int j = 0; j < _subdivisions; j++)
    {
        for (int i = 0; i < _subdivisions; i++)
        {
            /*
            int remove = 0;
            for (const auto it : _deletedColIndex)
            {
                if ((j == it.first || (j == it.first + 1 && j != -1) || (j == it.first - 1))
                    && (it.second == i || (it.second == i + 1) || (it.second == i - 1)))
                {
                    remove++;
                    // TODO mettre la 3eme boucle en dehors de la 2eme
                }
            }
            if (remove > 0)
            {
                indices[index++] = 0;
                indices[index++] = 0;
                indices[index++] = 0;

                indices[index++] = 0;
                indices[index++] = 0;
                indices[index++] = 0;
            }
            else*/
            {
                int topLeft = (i * (_subdivisions + 1)) + j;
                int topRight = topLeft + 1;
                int bottomLeft = topLeft + _subdivisions + 1;
                int bottomRight = bottomLeft + 1;

                indices[index++] = topLeft;
                indices[index++] = bottomLeft;
                indices[index++] = topRight;

                indices[index++] = topRight;
                indices[index++] = bottomLeft;
                indices[index++] = bottomRight;
            }
        }
    }

    _indices.clear();
    for (size_t i = 0; i < _indexCount; i++)
        _indices.append(indices[i]);
}

void Surface::fillVertices(QSGGeometry::TexturedPoint2D* vertices)
{
    _vertices.clear();
    for (int i = 0; i < _vertexCount; i++)
    {
        QPoint p(vertices[i].x, vertices[i].y);
        _vertices.append(p);
    }
        
    _verticesChanged = false;
    _reinit = false;
    Q_EMIT verticesChanged();
}

void Surface::drawGrid(QSGGeometry* geometryLine)
{
    removeGrid(geometryLine);

    if (_displayGrid)
    {
        int countPoint = 0;
        int index = 0;
        for (size_t i = 0; i <= _subdivisions; i++)
        {
            for (size_t j = 0; j <= _subdivisions; j++)
            {
                if (i == _subdivisions && j == _subdivisions)
                    continue;
                const auto& p = _vertices[index];
                // Horizontal Line
                if (i != _subdivisions)
                {
                    geometryLine->vertexDataAsPoint2D()[countPoint++].set(p.x(), p.y());
                    index += _subdivisions + 1;
                    geometryLine->vertexDataAsPoint2D()[countPoint++].set(p.x(), p.y());
                    index -= _subdivisions + 1;

                    if (j == _subdivisions)
                    {
                        index++;
                        continue;
                    }
                }

                // Vertical Line
                geometryLine->vertexDataAsPoint2D()[countPoint++].set(p.x(), p.y());
                index++;
                geometryLine->vertexDataAsPoint2D()[countPoint++].set(p.x(), p.y());
            }
        }
    }
    _gridChanged = false;
    Q_EMIT verticesChanged();
}

void Surface::setSubdivisions(int sub)
{
	_subdivisions = sub;
	
	// Update vertexCount and indexCount according to new subdivision count
	_vertexCount = (_subdivisions + 1) * (_subdivisions + 1);
	_indexCount = _subdivisions * _subdivisions * 6;
}

int Surface::subdivisions() const
{
	return _subdivisions;
}

void Surface::removeGrid(QSGGeometry* geometryLine)
{
    for (size_t i = 0; i < geometryLine->vertexCount(); i++)
    {
        geometryLine->vertexDataAsPoint2D()[i].set(0, 0);
    }
}

bool Surface::loadSfmData()
{
    using namespace aliceVision::sfmDataIO;

    // Clear sfmData
    _sfmData.clear();

    if (_sfmPath.isEmpty())
    {
        _loadedSfmPath.clear();
        return false;
    }

    qWarning() << "Surface::loadSfmData: '" << _sfmPath << "'.\n";
    // load SfMData files
    if (!Load(_sfmData, _sfmPath.toStdString(), ESfMData(ESfMData::VIEWS | ESfMData::EXTRINSICS | ESfMData::INTRINSICS)))
    {
        qWarning() << "The input SfMData file '" << _sfmPath << "' cannot be read.\n";
        return false;
    }

    if (_sfmData.getViews().empty())
    {
        qWarning() << "The input SfMData file '" << _sfmPath << "' is empty.\n";
        return false;
    }
    _loadedSfmPath = _sfmPath;

    return true;
}

void Surface::computePrincipalPoint(aliceVision::camera::IntrinsicBase* intrinsic, QSize textureSize)
{
    const aliceVision::Vec2 center(textureSize.width() * 0.5, textureSize.height() * 0.5);
    aliceVision::Vec2 ppCorrection(0.0, 0.0);

    if (aliceVision::camera::isPinhole(intrinsic->getType()))
    {
        ppCorrection = dynamic_cast<aliceVision::camera::IntrinsicsScaleOffset&>(*intrinsic).getOffset();
    }

    _principalPoint.setX(ppCorrection.x());
    _principalPoint.setY(ppCorrection.y());
}

void Surface::setRotationValues(float yaw, float pitch)
{
    _yaw = yaw;
    _pitch = pitch;
    _isPanoramaRotating = true;
}

void Surface::incrementRotationValues(float yaw, float pitch)
{
    _yaw += yaw;
    if (aliceVision::radianToDegree(_pitch + pitch) <= 90 && aliceVision::radianToDegree(_pitch + pitch) >= -90)
    {
        _pitch += pitch;
    }
    _isPanoramaRotating = true;
}

void Surface::rotatePanorama(aliceVision::Vec3& coordSphere)
{
    Eigen::AngleAxis<double> Myaw(_yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxis<double> Mpitch(_pitch, Eigen::Vector3d::UnitX());

    Eigen::Matrix3d cRo = Myaw.toRotationMatrix() * Mpitch.toRotationMatrix();

    coordSphere = cRo * coordSphere;
}

}  // ns qtAliceVision