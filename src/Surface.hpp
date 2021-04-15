#pragma once
#define _USE_MATH_DEFINES

#include <QQuickItem>
#include <QSGGeometry>
#include <QVariant>
#include <string>
#include <vector>

#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/IntrinsicBase.hpp>
#include <aliceVision/numeric/numeric.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmData/CameraPose.hpp>


namespace qtAliceVision
{

enum class ViewerType
{
	DEFAULT = 0, HDR, DISTORTION, PANORAMA
};

/**
 * @brief Discretization of FloatImageViewer surface
 */
class Surface : public QObject
{
Q_OBJECT
	Q_PROPERTY(QList<QPoint> vertices READ vertices NOTIFY verticesChanged);
	Q_PROPERTY(bool displayGrid READ getDisplayGrid WRITE setDisplayGrid NOTIFY displayGridChanged);
	Q_PROPERTY(int idView READ getIdView WRITE setIdView NOTIFY idViewChanged);
	Q_PROPERTY(QColor gridColor READ getGridColor WRITE setGridColor NOTIFY gridColorChanged);
	Q_PROPERTY(QString sfmPath READ getSfmPath WRITE setSfmPath NOTIFY sfmPathChanged);

public:
	Surface(int subdivisions = 4, QObject* parent = nullptr);
	Surface& operator=(const Surface& other) = default;
	~Surface();

public:
	Q_SIGNAL void verticesChanged();
	Q_SIGNAL void gridChanged();
	Q_SIGNAL void gridColorChanged();
	Q_SIGNAL void sfmChanged();
	Q_SIGNAL void sfmPathChanged();
	Q_SIGNAL void displayGridChanged();
	Q_SIGNAL void idViewChanged();

	Q_INVOKABLE QPoint getVertex(int index);
	Q_INVOKABLE void setVertex(int index, float x, float y);
	Q_INVOKABLE void displayGrid(bool display);
	Q_INVOKABLE void defaultControlPoints();
	Q_INVOKABLE bool reinit();
	Q_INVOKABLE void setupLensDistortion(bool distortion);
	Q_INVOKABLE void updateSubdivisions(int subs);
	Q_INVOKABLE QPoint getPrincipalPoint();
	Q_INVOKABLE void setPanoramaViewerEnabled(bool state);
	Q_INVOKABLE void rotatePanoramaRadians(float yawRadians, float pitchRadians);
	Q_INVOKABLE void rotatePanoramaDegrees(float yawDegrees, float pitchDegrees);
	Q_INVOKABLE void mouseOver(bool state);
	Q_INVOKABLE double getPitchDegree();
	Q_INVOKABLE double getYawDegree();
	Q_INVOKABLE bool isMouseInside(float mx, float my);

public:
	bool update(QSGGeometry::TexturedPoint2D* vertices, quint16* indices, QSize textureSize, bool updateSfmData);

	void fillVertices(QSGGeometry::TexturedPoint2D* vertices);

	void drawGrid(QSGGeometry* geometryLine);

	void removeGrid(QSGGeometry* geometryLine);

	QPoint principalPoint() const { return _principalPoint; }

	double pitch() const { return _pitch; }
	double yaw() const { return _yaw; }

	void setRotationValues(float yaw, float pitch);
	void incrementRotationValues(float yaw, float pitch);

	void setSubdivisions(int sub);
	int subdivisions() const;

	const QList<QPoint>& vertices() const { return _vertices; }
	void clearVertices() { _vertices.clear(); _coordsSphereDefault.clear(); }
		
	const QPoint& vertex(int index) const { return _vertices[index]; }
	QPoint& vertex(int index) { return _vertices[index]; }
	const quint32 index(int index) { return _indices[index]; }

	inline int indexCount() const { return _indexCount; }
	inline int vertexCount() const { return _vertexCount; }

	inline QColor getGridColor() const { return _gridColor; }
	inline void setGridColor(const QColor& color)
	{
		_gridColor = color;
		Q_EMIT gridColorChanged();
	}

	inline bool hasVerticesChanged() const { return _verticesChanged; }
	void setVerticesChanged(bool change)
	{
		// if (_verticesChanged == change)
		//     return;
		_verticesChanged = change;
		Q_EMIT verticesChanged();
	}

	inline bool hasGridChanged() const { return _gridChanged; }
	void setGridChanged(bool change)
	{
		// if(_gridChanged == change)
		//    return;
		_gridChanged = change;
		// Q_EMIT gridChanged();
	}

	void setDisplayGrid(bool display) { _displayGrid = display; }
	bool getDisplayGrid() const { return _displayGrid; }

	void reinitialize(bool reinit) { _reinit = reinit; }
	bool hasReinitialized() const { return _reinit; }

	bool hasSubsChanged() { return _subsChanged; }
	void setSubsChanged(bool change) { _subsChanged = change; }

	void setSfmPath(const QString& path)
	{
		if(_sfmPath == path)
			return;
		_loadedSfmPath = _sfmPath;
		_sfmPath = path;
		Q_EMIT sfmPathChanged();
	}
	QString getSfmPath() const { return _sfmPath; }

	void setIdView(aliceVision::IndexT id) { _idView = id; }
	aliceVision::IndexT getIdView() const { return _idView; }

	bool isMouseOver() const { return _mouseOver; }
	void setMouseOver(bool state) { _mouseOver = state; }

	// Viewer Type
	ViewerType viewerType() const { return _viewerType; }
	void setViewerType(ViewerType type) { _viewerType = type; }

	bool isPanoramaViewerEnabled() const;
	bool isLensDistortionViewerEnabled() const;

private:
	bool loadSfmData();

	void computeGrid(QSGGeometry::TexturedPoint2D* vertices, quint16* indices, QSize textureSize, bool updateSfm);

	void computeVerticesGrid(QSGGeometry::TexturedPoint2D* vertices, QSize textureSize,
		aliceVision::camera::IntrinsicBase* intrinsic);

	void computeIndicesGrid(quint16* indices);

	void computePrincipalPoint(aliceVision::camera::IntrinsicBase* intrinsic, QSize textureSize);

	void rotatePanorama(aliceVision::Vec3& coordSphere);

private:
	int _panoramaWidth = 3000;
	int _panoramaHeight = 1500;

	// Vertex Data
	QList<QPoint> _vertices;
	QList<quint16> _indices;
	int _subdivisions;
	int _vertexCount;
	int _indexCount;
	std::vector<std::pair<int, int> > _deletedColIndex;

	// Vertices State
	bool _verticesChanged = true;
	bool _reinit = false;

	// Grid State
	bool _displayGrid = false;
	bool _gridChanged = true;
	QColor _gridColor = QColor(255, 0, 0, 255);
	bool _subsChanged = false;

	// Sfm Data
	aliceVision::sfmData::SfMData _sfmData;
	QString _loadedSfmPath;
	QString _sfmPath;

	// Principal Point Coord
	QPoint _principalPoint = QPoint(0, 0);

	// Id View
	aliceVision::IndexT _idView;

	// Viewer
	ViewerType _viewerType = ViewerType::DEFAULT;

	// Euler angle in radians
	double _pitch = 0.0;
	double _yaw = 0.0;

	// Coordinates on Unit Sphere without any rotation
	std::vector<aliceVision::Vec3> _coordsSphereDefault;
	// Mouse Over 
	bool _mouseOver = false;
	// If panorama is currently rotating
	bool _isPanoramaRotating = false;
	// Mouse Area Coordinates
	QVariantList _mouseAreaCoords = { 0, 0, 0, 0 };
};

}  // ns qtAliceVision

// Q_DECLARE_METATYPE(qtAliceVision::Surface*);

