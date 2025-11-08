#include <QApplication>
#include <QMainWindow>
#include <QPushButton>
#include <QFileDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QMessageBox>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QProgressBar>
#include <QStatusBar>
#include <QComboBox>
#include <QTabWidget>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QListWidget>
#include <QInputDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QTextStream>

#include <string>
#include <sstream>
#include <cmath>

#include <pdal/pdal.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/filters/OutlierFilter.hpp>
#include <pdal/filters/DecimationFilter.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/poisson.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkJPEGWriter.h>
#include <vtkTIFFWriter.h>
#include <vtkImageWriter.h>
#include <vtkPoints.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPolyData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkCamera.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkCellArray.h>
#include <vtkTriangle.h>

// Constants
const double MIN_AXIS_LENGTH = 0.001;  // Minimum distance between two axis points

// Structure to store elevation orthophoto parameters
struct ElevationOrthoParams {
    double point1_x, point1_y;  // First point defining the axis
    double point2_x, point2_y;  // Second point defining the axis
    double start_z;              // Starting Z
    double height;               // Box height
    double depth;                // Box depth
    bool depth_forward;          // Depth direction (true = forward, false = backward)
    double resolution;           // Resolution in pixels per meter
};

// Structure to store plan orthophoto parameters
struct PlanOrthoParams {
    double corner_x, corner_y;   // Lower corner of the box
    double cutting_height_z;     // Cutting height
    double width;                // Frame width
    double height;               // Frame height
    double depth;                // Orthophoto depth
    bool depth_up;               // Depth direction (true = up, false = down)
    double resolution;           // Resolution in pixels per meter
};

// Structure to store orthophoto corners (legacy system)
struct OrthoCorners {
    double x_min, y_min, z_min;
    double x_max, y_max, z_max;
};

// Enum for orthophoto mode type
enum class OrthoMode {
    Elevation,
    Plan,
    Legacy
};

// Structure to store a complete orthophoto configuration for batch processing
struct OrthoConfig {
    QString name;
    OrthoMode mode;
    QString outputPath;
    
    // Elevation parameters
    ElevationOrthoParams elevationParams;
    
    // Plan parameters
    PlanOrthoParams planParams;
    
    // Legacy parameters
    OrthoCorners legacyCorners;
};

class OrthophotoGUI : public QMainWindow {
    Q_OBJECT

public:
    OrthophotoGUI(QWidget *parent = nullptr) : QMainWindow(parent) {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        setupUI();
        setupMenuBar();
        setupConnections();
        setWindowTitle("Orthophoto & Mesh Tool 2.0");
        resize(1000, 800);
    }

private:
    // UI elements
    QPushButton *selectFileBtn;
    QPushButton *preprocessBtn;
    QPushButton *meshBtn;
    QPushButton *exportBtn;
    QPushButton *visualizeBtn;
    QLabel *fileLabel;
    QLabel *statusLabel;
    QProgressBar *progressBar;
    QDoubleSpinBox *resolutionSpinBox;
    QCheckBox *colorMeshCheckBox;
    
    // Tab widget for orthophoto modes
    QTabWidget *orthoModeTab;
    
    // Elevation orthophoto parameters
    QDoubleSpinBox *elev_p1x, *elev_p1y;
    QDoubleSpinBox *elev_p2x, *elev_p2y;
    QDoubleSpinBox *elev_start_z, *elev_height, *elev_depth;
    QComboBox *elev_depth_direction;
    QDoubleSpinBox *elev_resolution;
    
    // Plan orthophoto parameters
    QDoubleSpinBox *plan_corner_x, *plan_corner_y;
    QDoubleSpinBox *plan_cutting_z;
    QDoubleSpinBox *plan_width, *plan_height, *plan_depth;
    QComboBox *plan_depth_direction;
    QDoubleSpinBox *plan_resolution;
    
    // Legacy corner system (for compatibility)
    QGroupBox *cornersGroup;
    QDoubleSpinBox *xMinSpinBox, *yMinSpinBox, *zMinSpinBox;
    QDoubleSpinBox *xMaxSpinBox, *yMaxSpinBox, *zMaxSpinBox;

    // Filtres PDAL
    QCheckBox *outlierFilterCheckBox;
    QDoubleSpinBox *outlierMultiplierSpinBox;
    QCheckBox *decimationFilterCheckBox;
    QSpinBox *decimationStepSpinBox;

    // Batch processing UI
    QListWidget *batchListWidget;
    QPushButton *addToBatchBtn;
    QPushButton *removeFromBatchBtn;
    QPushButton *processBatchBtn;
    QPushButton *clearBatchBtn;

    // Data
    QString currentFile;
    QString currentWorkFile;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PolygonMesh mesh;
    OrthoCorners orthoCorners;
    ElevationOrthoParams elevationParams;
    PlanOrthoParams planParams;
    
    // Batch processing data
    QList<OrthoConfig> batchConfigs;

    // VTK pour la visualisation
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkActor> actor;

    // Helper function to convert PCL mesh to VTK polydata
    void convertPCLMeshToVTK(const pcl::PolygonMesh& pclMesh, vtkSmartPointer<vtkPolyData> vtkPolyData) {
        // Extract points
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(pclMesh.cloud, *cloud);
        
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3);
        colors->SetName("Colors");
        
        for (const auto& point : cloud->points) {
            points->InsertNextPoint(point.x, point.y, point.z);
            unsigned char color[3] = {
                static_cast<unsigned char>(point.r),
                static_cast<unsigned char>(point.g),
                static_cast<unsigned char>(point.b)
            };
            colors->InsertNextTuple3(color[0], color[1], color[2]);
        }
        
        // Extract polygons
        vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
        for (const auto& polygon : pclMesh.polygons) {
            vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
            if (polygon.vertices.size() == 3) {
                triangle->GetPointIds()->SetId(0, polygon.vertices[0]);
                triangle->GetPointIds()->SetId(1, polygon.vertices[1]);
                triangle->GetPointIds()->SetId(2, polygon.vertices[2]);
                polygons->InsertNextCell(triangle);
            }
        }
        
        vtkPolyData->SetPoints(points);
        vtkPolyData->SetPolys(polygons);
        vtkPolyData->GetPointData()->SetScalars(colors);
    }

    // Function to setup menu bar
    void setupMenuBar() {
        QMenuBar *menuBar = new QMenuBar(this);
        setMenuBar(menuBar);
        
        // File menu
        QMenu *fileMenu = menuBar->addMenu("Fichier");
        
        QAction *newAction = fileMenu->addAction("Nouveau");
        connect(newAction, &QAction::triggered, this, &OrthophotoGUI::newWorkFile);
        
        QAction *openAction = fileMenu->addAction("Ouvrir...");
        connect(openAction, &QAction::triggered, this, &OrthophotoGUI::openWorkFile);
        
        QAction *saveAction = fileMenu->addAction("Enregistrer");
        connect(saveAction, &QAction::triggered, this, &OrthophotoGUI::saveWorkFile);
        
        QAction *saveAsAction = fileMenu->addAction("Enregistrer sous...");
        connect(saveAsAction, &QAction::triggered, this, &OrthophotoGUI::saveWorkFileAs);
        
        fileMenu->addSeparator();
        
        QAction *exitAction = fileMenu->addAction("Quitter");
        connect(exitAction, &QAction::triggered, this, &QWidget::close);
    }

    // Function to configure the user interface
    void setupUI() {
        QWidget *centralWidget = new QWidget(this);
        QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);

        // Main buttons
        selectFileBtn = new QPushButton("Sélectionner nuage de points (.e57, .las, .laz)", this);
        preprocessBtn = new QPushButton("Pré-traitement PDAL", this);
        meshBtn = new QPushButton("Générer Mesh Poisson", this);
        exportBtn = new QPushButton("Exporter Orthophoto", this);
        visualizeBtn = new QPushButton("Visualiser", this);

        // Étiquettes et champs
        fileLabel = new QLabel("Aucun fichier sélectionné", this);
        statusLabel = new QLabel("Prêt", this);
        progressBar = new QProgressBar(this);
        progressBar->setRange(0, 100);

        // Options de mesh
        QGroupBox *meshGroup = new QGroupBox("Options de Mesh", this);
        QVBoxLayout *meshLayout = new QVBoxLayout(meshGroup);
        resolutionSpinBox = new QDoubleSpinBox(this);
        resolutionSpinBox->setRange(1, 20);
        resolutionSpinBox->setValue(8);
        resolutionSpinBox->setSuffix(" (profondeur)");
        colorMeshCheckBox = new QCheckBox("Colorier le mesh", this);
        colorMeshCheckBox->setChecked(true);
        meshLayout->addWidget(new QLabel("Profondeur Poisson:", this));
        meshLayout->addWidget(resolutionSpinBox);
        meshLayout->addWidget(colorMeshCheckBox);

        // Options de filtrage PDAL
        QGroupBox *pdalGroup = new QGroupBox("Filtres PDAL", this);
        QVBoxLayout *pdalLayout = new QVBoxLayout(pdalGroup);
        
        outlierFilterCheckBox = new QCheckBox("Filtrer les outliers", this);
        outlierFilterCheckBox->setChecked(true);
        pdalLayout->addWidget(outlierFilterCheckBox);
        
        QHBoxLayout *outlierLayout = new QHBoxLayout();
        outlierLayout->addWidget(new QLabel("Multiplicateur:", this));
        outlierMultiplierSpinBox = new QDoubleSpinBox(this);
        outlierMultiplierSpinBox->setRange(0.5, 10.0);
        outlierMultiplierSpinBox->setValue(2.0);
        outlierMultiplierSpinBox->setSingleStep(0.5);
        outlierLayout->addWidget(outlierMultiplierSpinBox);
        pdalLayout->addLayout(outlierLayout);
        
        decimationFilterCheckBox = new QCheckBox("Décimation du nuage", this);
        decimationFilterCheckBox->setChecked(false);
        pdalLayout->addWidget(decimationFilterCheckBox);
        
        QHBoxLayout *decimationLayout = new QHBoxLayout();
        decimationLayout->addWidget(new QLabel("Pas (1 point sur N):", this));
        decimationStepSpinBox = new QSpinBox(this);
        decimationStepSpinBox->setRange(2, 100);
        decimationStepSpinBox->setValue(10);
        decimationLayout->addWidget(decimationStepSpinBox);
        pdalLayout->addLayout(decimationLayout);

        // Tabs pour les modes d'orthophoto
        orthoModeTab = new QTabWidget(this);
        
        // Tab 1: Elevation Orthophoto
        QWidget *elevationTab = new QWidget();
        QGridLayout *elevLayout = new QGridLayout(elevationTab);
        
        elevLayout->addWidget(new QLabel("Point 1 - X:", this), 0, 0);
        elev_p1x = new QDoubleSpinBox(this);
        elev_p1x->setRange(-1000000, 1000000);
        elev_p1x->setDecimals(3);
        elevLayout->addWidget(elev_p1x, 0, 1);
        
        elevLayout->addWidget(new QLabel("Point 1 - Y:", this), 0, 2);
        elev_p1y = new QDoubleSpinBox(this);
        elev_p1y->setRange(-1000000, 1000000);
        elev_p1y->setDecimals(3);
        elevLayout->addWidget(elev_p1y, 0, 3);
        
        elevLayout->addWidget(new QLabel("Point 2 - X:", this), 1, 0);
        elev_p2x = new QDoubleSpinBox(this);
        elev_p2x->setRange(-1000000, 1000000);
        elev_p2x->setDecimals(3);
        elevLayout->addWidget(elev_p2x, 1, 1);
        
        elevLayout->addWidget(new QLabel("Point 2 - Y:", this), 1, 2);
        elev_p2y = new QDoubleSpinBox(this);
        elev_p2y->setRange(-1000000, 1000000);
        elev_p2y->setDecimals(3);
        elevLayout->addWidget(elev_p2y, 1, 3);
        
        elevLayout->addWidget(new QLabel("Z de départ:", this), 2, 0);
        elev_start_z = new QDoubleSpinBox(this);
        elev_start_z->setRange(-1000000, 1000000);
        elev_start_z->setDecimals(3);
        elevLayout->addWidget(elev_start_z, 2, 1);
        
        elevLayout->addWidget(new QLabel("Hauteur:", this), 2, 2);
        elev_height = new QDoubleSpinBox(this);
        elev_height->setRange(0.1, 10000);
        elev_height->setValue(10.0);
        elev_height->setDecimals(3);
        elevLayout->addWidget(elev_height, 2, 3);
        
        elevLayout->addWidget(new QLabel("Profondeur:", this), 3, 0);
        elev_depth = new QDoubleSpinBox(this);
        elev_depth->setRange(0.1, 10000);
        elev_depth->setValue(5.0);
        elev_depth->setDecimals(3);
        elevLayout->addWidget(elev_depth, 3, 1);
        
        elevLayout->addWidget(new QLabel("Sens profondeur:", this), 3, 2);
        elev_depth_direction = new QComboBox(this);
        elev_depth_direction->addItem("Avant");
        elev_depth_direction->addItem("Arrière");
        elevLayout->addWidget(elev_depth_direction, 3, 3);
        
        elevLayout->addWidget(new QLabel("Résolution (px/m):", this), 4, 0);
        elev_resolution = new QDoubleSpinBox(this);
        elev_resolution->setRange(1, 1000);
        elev_resolution->setValue(100.0);
        elev_resolution->setDecimals(1);
        elevLayout->addWidget(elev_resolution, 4, 1);
        
        orthoModeTab->addTab(elevationTab, "Orthophoto Élévation");
        
        // Tab 2: Orthophoto en plan
        QWidget *planTab = new QWidget();
        QGridLayout *planLayout = new QGridLayout(planTab);
        
        planLayout->addWidget(new QLabel("Coin bas - X:", this), 0, 0);
        plan_corner_x = new QDoubleSpinBox(this);
        plan_corner_x->setRange(-1000000, 1000000);
        plan_corner_x->setDecimals(3);
        planLayout->addWidget(plan_corner_x, 0, 1);
        
        planLayout->addWidget(new QLabel("Coin bas - Y:", this), 0, 2);
        plan_corner_y = new QDoubleSpinBox(this);
        plan_corner_y->setRange(-1000000, 1000000);
        plan_corner_y->setDecimals(3);
        planLayout->addWidget(plan_corner_y, 0, 3);
        
        planLayout->addWidget(new QLabel("Hauteur de coupe Z:", this), 1, 0);
        plan_cutting_z = new QDoubleSpinBox(this);
        plan_cutting_z->setRange(-1000000, 1000000);
        plan_cutting_z->setDecimals(3);
        planLayout->addWidget(plan_cutting_z, 1, 1);
        
        planLayout->addWidget(new QLabel("Largeur cadre:", this), 2, 0);
        plan_width = new QDoubleSpinBox(this);
        plan_width->setRange(0.1, 10000);
        plan_width->setValue(10.0);
        plan_width->setDecimals(3);
        planLayout->addWidget(plan_width, 2, 1);
        
        planLayout->addWidget(new QLabel("Hauteur cadre:", this), 2, 2);
        plan_height = new QDoubleSpinBox(this);
        plan_height->setRange(0.1, 10000);
        plan_height->setValue(10.0);
        plan_height->setDecimals(3);
        planLayout->addWidget(plan_height, 2, 3);
        
        planLayout->addWidget(new QLabel("Profondeur:", this), 3, 0);
        plan_depth = new QDoubleSpinBox(this);
        plan_depth->setRange(0.1, 10000);
        plan_depth->setValue(5.0);
        plan_depth->setDecimals(3);
        planLayout->addWidget(plan_depth, 3, 1);
        
        planLayout->addWidget(new QLabel("Sens profondeur:", this), 3, 2);
        plan_depth_direction = new QComboBox(this);
        plan_depth_direction->addItem("Vers le haut");
        plan_depth_direction->addItem("Vers le bas");
        planLayout->addWidget(plan_depth_direction, 3, 3);
        
        planLayout->addWidget(new QLabel("Résolution (px/m):", this), 4, 0);
        plan_resolution = new QDoubleSpinBox(this);
        plan_resolution->setRange(1, 1000);
        plan_resolution->setValue(100.0);
        plan_resolution->setDecimals(1);
        planLayout->addWidget(plan_resolution, 4, 1);
        
        orthoModeTab->addTab(planTab, "Orthophoto Plan");
        
        // Tab 3: Legacy corner system (for compatibility)
        QWidget *cornersTab = new QWidget();
        QGridLayout *cornersLayout = new QGridLayout(cornersTab);

        cornersLayout->addWidget(new QLabel("X min:", this), 0, 0);
        xMinSpinBox = new QDoubleSpinBox(this);
        xMinSpinBox->setRange(-1000000, 1000000);
        cornersLayout->addWidget(xMinSpinBox, 0, 1);

        cornersLayout->addWidget(new QLabel("Y min:", this), 0, 2);
        yMinSpinBox = new QDoubleSpinBox(this);
        yMinSpinBox->setRange(-1000000, 1000000);
        cornersLayout->addWidget(yMinSpinBox, 0, 3);

        cornersLayout->addWidget(new QLabel("Z min:", this), 0, 4);
        zMinSpinBox = new QDoubleSpinBox(this);
        zMinSpinBox->setRange(-1000000, 1000000);
        cornersLayout->addWidget(zMinSpinBox, 0, 5);

        cornersLayout->addWidget(new QLabel("X max:", this), 1, 0);
        xMaxSpinBox = new QDoubleSpinBox(this);
        xMaxSpinBox->setRange(-1000000, 1000000);
        cornersLayout->addWidget(xMaxSpinBox, 1, 1);

        cornersLayout->addWidget(new QLabel("Y max:", this), 1, 2);
        yMaxSpinBox = new QDoubleSpinBox(this);
        yMaxSpinBox->setRange(-1000000, 1000000);
        cornersLayout->addWidget(yMaxSpinBox, 1, 3);

        cornersLayout->addWidget(new QLabel("Z max:", this), 1, 4);
        zMaxSpinBox = new QDoubleSpinBox(this);
        zMaxSpinBox->setRange(-1000000, 1000000);
        cornersLayout->addWidget(zMaxSpinBox, 1, 5);
        
        orthoModeTab->addTab(cornersTab, "Coins XYZ (Ancien)");
        
        // Tab 4: Batch Mode
        QWidget *batchTab = new QWidget();
        QVBoxLayout *batchLayout = new QVBoxLayout(batchTab);
        
        QLabel *batchLabel = new QLabel("Gestion des orthophotos en batch:", this);
        batchLayout->addWidget(batchLabel);
        
        batchListWidget = new QListWidget(this);
        batchLayout->addWidget(batchListWidget);
        
        QHBoxLayout *batchButtonLayout = new QHBoxLayout();
        addToBatchBtn = new QPushButton("Ajouter au batch", this);
        removeFromBatchBtn = new QPushButton("Retirer du batch", this);
        clearBatchBtn = new QPushButton("Vider le batch", this);
        batchButtonLayout->addWidget(addToBatchBtn);
        batchButtonLayout->addWidget(removeFromBatchBtn);
        batchButtonLayout->addWidget(clearBatchBtn);
        batchLayout->addLayout(batchButtonLayout);
        
        processBatchBtn = new QPushButton("Traiter le batch", this);
        processBatchBtn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }");
        batchLayout->addWidget(processBatchBtn);
        
        orthoModeTab->addTab(batchTab, "Batch");

        // Add elements to main layout
        mainLayout->addWidget(selectFileBtn);
        mainLayout->addWidget(fileLabel);
        mainLayout->addWidget(pdalGroup);
        mainLayout->addWidget(preprocessBtn);
        mainLayout->addWidget(meshGroup);
        mainLayout->addWidget(meshBtn);
        mainLayout->addWidget(orthoModeTab);
        mainLayout->addWidget(exportBtn);
        mainLayout->addWidget(visualizeBtn);
        mainLayout->addWidget(progressBar);
        mainLayout->addWidget(statusLabel);

        centralWidget->setLayout(mainLayout);
        setCentralWidget(centralWidget);

        // Barre de statut
        statusBar()->showMessage("Prêt");
    }

    // Function to connect signals and slots
    void setupConnections() {
        connect(selectFileBtn, &QPushButton::clicked, this, &OrthophotoGUI::selectFile);
        connect(preprocessBtn, &QPushButton::clicked, this, &OrthophotoGUI::preprocessPDAL);
        connect(meshBtn, &QPushButton::clicked, this, &OrthophotoGUI::generateMesh);
        connect(exportBtn, &QPushButton::clicked, this, &OrthophotoGUI::exportOrthophoto);
        connect(visualizeBtn, &QPushButton::clicked, this, &OrthophotoGUI::visualize);
        
        // Batch processing connections
        connect(addToBatchBtn, &QPushButton::clicked, this, &OrthophotoGUI::addToBatch);
        connect(removeFromBatchBtn, &QPushButton::clicked, this, &OrthophotoGUI::removeFromBatch);
        connect(clearBatchBtn, &QPushButton::clicked, this, &OrthophotoGUI::clearBatch);
        connect(processBatchBtn, &QPushButton::clicked, this, &OrthophotoGUI::processBatch);
    }

private slots:
    // 1. Select a point cloud file
    void selectFile() {
        QString fileName = QFileDialog::getOpenFileName(this,
            "Ouvrir nuage de points",
            "",
            "E57 Files (*.e57);;LAS/LAZ Files (*.las *.laz);;All Files (*.*)");

        if (!fileName.isEmpty()) {
            currentFile = fileName;
            fileLabel->setText("Fichier: " + fileName);
            statusBar()->showMessage("Fichier chargé: " + fileName);
            QMessageBox::information(this, "Fichier sélectionné", "Fichier: " + fileName);
        }
    }

    // 2. PDAL preprocessing (filtering, cleaning, noise reduction)
    void preprocessPDAL() {
        if (currentFile.isEmpty()) {
            QMessageBox::warning(this, "Erreur", "Aucun fichier sélectionné!");
            return;
        }

        statusLabel->setText("Pré-traitement en cours...");
        progressBar->setValue(0);
        qApp->processEvents();

        try {
            // Determine reader type based on file extension
            std::string readerType = "readers.las";
            std::string filename = currentFile.toStdString();
            if (filename.find(".e57") != std::string::npos) {
                readerType = "readers.e57";
            } else if (filename.find(".laz") != std::string::npos) {
                readerType = "readers.las";
            }
            
            // Build PDAL pipeline with optimized settings
            std::ostringstream pipelineJson;
            pipelineJson << "{\"pipeline\":[";
            pipelineJson << "{\"type\":\"" << readerType << "\",\"filename\":\"" << filename << "\"}";
            
            // Add outlier filter if enabled
            if (outlierFilterCheckBox->isChecked()) {
                pipelineJson << ",{\"type\":\"filters.outlier\",\"method\":\"statistical\",\"mean_k\":8,\"multiplier\":" 
                            << outlierMultiplierSpinBox->value() << "}";
            }
            
            // Add decimation filter if enabled
            if (decimationFilterCheckBox->isChecked()) {
                pipelineJson << ",{\"type\":\"filters.decimation\",\"step\":" 
                            << decimationStepSpinBox->value() << "}";
            }
            
            pipelineJson << ",{\"type\":\"writers.pcd\",\"filename\":\"output.pcd\"}]}";
            
            pdal::PipelineManager manager;
            std::istringstream iss(pipelineJson.str());
            manager.readPipeline(iss);
            manager.execute();

            progressBar->setValue(50);
            qApp->processEvents();

            // Load point cloud into PCL
            cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
            if (pcl::io::loadPCDFile("output.pcd", *cloud) == -1) {
                throw std::runtime_error("Unable to load point cloud after PDAL processing.");
            }

            // Validate cloud is not empty
            if (cloud->empty()) {
                throw std::runtime_error("Point cloud is empty after PDAL processing.");
            }

            // Update orthophoto corners (approximation)
            pcl::PointXYZRGB minPt, maxPt;
            pcl::getMinMax3D(*cloud, minPt, maxPt);
            
            // Update legacy system
            xMinSpinBox->setValue(minPt.x);
            yMinSpinBox->setValue(minPt.y);
            zMinSpinBox->setValue(minPt.z);
            xMaxSpinBox->setValue(maxPt.x);
            yMaxSpinBox->setValue(maxPt.y);
            zMaxSpinBox->setValue(maxPt.z);
            
            // Calculate center and range
            const double center_x = (minPt.x + maxPt.x) * 0.5;
            const double center_y = (minPt.y + maxPt.y) * 0.5;
            const double range_x = maxPt.x - minPt.x;
            const double range_y = maxPt.y - minPt.y;
            
            // Update elevation parameters (suggestions)
            elev_p1x->setValue(center_x - range_x * 0.25);
            elev_p1y->setValue(center_y);
            elev_p2x->setValue(center_x + range_x * 0.25);
            elev_p2y->setValue(center_y);
            elev_start_z->setValue(minPt.z);
            elev_height->setValue(maxPt.z - minPt.z);
            
            // Update plan parameters (suggestions)
            plan_corner_x->setValue(minPt.x);
            plan_corner_y->setValue(minPt.y);
            plan_cutting_z->setValue((minPt.z + maxPt.z) * 0.5);
            plan_width->setValue(range_x);
            plan_height->setValue(range_y);

            statusLabel->setText("Pré-traitement terminé. " + QString::number(cloud->points.size()) + " points chargés.");
            progressBar->setValue(100);
            statusBar()->showMessage("Pré-traitement terminé avec succès.");
        } catch (const std::exception& e) {
            QMessageBox::critical(this, "Erreur", "Erreur lors du pré-traitement: " + QString(e.what()));
            statusLabel->setText("Erreur: " + QString(e.what()));
            progressBar->setValue(0);
        }
    }

    // 3. Generate mesh with Poisson algorithm
    void generateMesh() {
        if (!cloud || cloud->empty()) {
            QMessageBox::warning(this, "Erreur", "Aucun nuage de points chargé!");
            return;
        }

        statusLabel->setText("Génération du mesh en cours...");
        progressBar->setValue(0);
        qApp->processEvents();

        try {
            // Convert to XYZ point cloud for normal estimation
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
            cloud_xyz->reserve(cloud->size());
            pcl::copyPointCloud(*cloud, *cloud_xyz);
            
            progressBar->setValue(25);
            qApp->processEvents();
            
            // Normal estimation with optimized parameters
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud_xyz);
            ne.setInputCloud(cloud_xyz);
            ne.setSearchMethod(tree);
            ne.setKSearch(20);
            ne.compute(*normals);
            
            progressBar->setValue(40);
            qApp->processEvents();
            
            // Create point cloud with normals
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields(*cloud_xyz, *normals, *cloud_with_normals);
            
            progressBar->setValue(50);
            qApp->processEvents();

            // Poisson reconstruction
            pcl::Poisson<pcl::PointNormal> poisson;
            poisson.setDepth(resolutionSpinBox->value());
            poisson.setInputCloud(cloud_with_normals);
            poisson.reconstruct(mesh);

            progressBar->setValue(75);
            qApp->processEvents();

            if (colorMeshCheckBox->isChecked()) {
                // Apply point cloud colors to mesh vertices
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::fromPCLPointCloud2(mesh.cloud, *mesh_cloud);
                
                pcl::search::KdTree<pcl::PointXYZRGB>::Ptr color_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
                color_tree->setInputCloud(cloud);
                
                // Pre-allocate vectors for better performance
                std::vector<int> indices(1);
                std::vector<float> distances(1);
                
                for (auto& point : mesh_cloud->points) {
                    if (color_tree->nearestKSearch(point, 1, indices, distances) > 0) {
                        const auto& src = cloud->points[indices[0]];
                        point.r = src.r;
                        point.g = src.g;
                        point.b = src.b;
                    }
                }
                
                pcl::toPCLPointCloud2(*mesh_cloud, mesh.cloud);
            }

            statusLabel->setText("Mesh généré avec " + QString::number(mesh.polygons.size()) + " polygones.");
            progressBar->setValue(100);
            statusBar()->showMessage("Mesh généré avec succès.");
        } catch (const std::exception& e) {
            QMessageBox::critical(this, "Erreur", "Erreur lors de la génération du mesh: " + QString(e.what()));
            statusLabel->setText("Erreur: " + QString(e.what()));
            progressBar->setValue(0);
        }
    }

    // 4. Export orthophoto
    void exportOrthophoto() {
        if (mesh.cloud.data.empty() && (!cloud || cloud->empty())) {
            QMessageBox::warning(this, "Erreur", "Aucun mesh ou nuage de points généré!");
            return;
        }

        QString fileName = QFileDialog::getSaveFileName(this,
            "Enregistrer l'orthophoto",
            "",
            "PNG Files (*.png);;JPEG Files (*.jpg);;TIFF Files (*.tif)");

        if (!fileName.isEmpty()) {
            statusLabel->setText("Export de l'orthophoto en cours...");
            progressBar->setValue(0);
            qApp->processEvents();

            try {
                // Determine active orthophoto mode
                int currentTab = orthoModeTab->currentIndex();
                
                if (currentTab == 0) {
                    // Elevation mode
                    exportElevationOrthophoto(fileName);
                } else if (currentTab == 1) {
                    // Plan mode
                    exportPlanOrthophoto(fileName);
                } else {
                    // Legacy mode (XYZ corners)
                    exportLegacyOrthophoto(fileName);
                }

                statusLabel->setText("Orthophoto exportée.");
                progressBar->setValue(100);
                statusBar()->showMessage("Orthophoto exportée avec succès: " + fileName);
            } catch (const std::exception& e) {
                QMessageBox::critical(this, "Erreur", "Erreur lors de l'export: " + QString(e.what()));
                statusLabel->setText("Erreur: " + QString(e.what()));
                progressBar->setValue(0);
            }
        }
    }
    
    void exportElevationOrthophoto(const QString& fileName) {
        // Get parameters
        elevationParams.point1_x = elev_p1x->value();
        elevationParams.point1_y = elev_p1y->value();
        elevationParams.point2_x = elev_p2x->value();
        elevationParams.point2_y = elev_p2y->value();
        elevationParams.start_z = elev_start_z->value();
        elevationParams.height = elev_height->value();
        elevationParams.depth = elev_depth->value();
        elevationParams.depth_forward = (elev_depth_direction->currentIndex() == 0);
        elevationParams.resolution = elev_resolution->value();
        
        // Configure VTK for orthographic rendering
        vtkSmartPointer<vtkGenericOpenGLRenderWindow> orthoRenderWindow = 
            vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
        vtkSmartPointer<vtkRenderer> orthoRenderer = vtkSmartPointer<vtkRenderer>::New();
        orthoRenderWindow->AddRenderer(orthoRenderer);
        orthoRenderer->SetBackground(1.0, 1.0, 1.0);
        
        // Calculate axis direction
        double dx = elevationParams.point2_x - elevationParams.point1_x;
        double dy = elevationParams.point2_y - elevationParams.point1_y;
        double axis_length = std::sqrt(dx*dx + dy*dy);
        
        if (axis_length < MIN_AXIS_LENGTH) {
            throw std::runtime_error("The two points are too close together");
        }
        
        // Normalized direction vector
        double dir_x = dx / axis_length;
        double dir_y = dy / axis_length;
        
        // Perpendicular vector (for depth)
        double perp_x = -dir_y;
        double perp_y = dir_x;
        if (!elevationParams.depth_forward) {
            perp_x = -perp_x;
            perp_y = -perp_y;
        }
        
        // Add data (mesh or point cloud)
        if (!mesh.cloud.data.empty()) {
            vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
            convertPCLMeshToVTK(mesh, polyData);
            
            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(polyData);
            
            vtkSmartPointer<vtkActor> orthoActor = vtkSmartPointer<vtkActor>::New();
            orthoActor->SetMapper(mapper);
            orthoRenderer->AddActor(orthoActor);
        } else if (cloud && !cloud->empty()) {
            addCloudToRenderer(orthoRenderer, cloud);
        }
        
        // Configure camera for elevation view
        vtkSmartPointer<vtkCamera> camera = orthoRenderer->GetActiveCamera();
        
        // Camera position (perpendicular to axis)
        double center_x = (elevationParams.point1_x + elevationParams.point2_x) / 2.0;
        double center_y = (elevationParams.point1_y + elevationParams.point2_y) / 2.0;
        double center_z = elevationParams.start_z + elevationParams.height / 2.0;
        
        double cam_dist = elevationParams.depth * 2.0;
        camera->SetPosition(
            center_x + perp_x * cam_dist,
            center_y + perp_y * cam_dist,
            center_z
        );
        camera->SetFocalPoint(center_x, center_y, center_z);
        camera->SetViewUp(0, 0, 1);
        camera->ParallelProjectionOn();
        
        // Calculate image size based on resolution
        int img_width = static_cast<int>(axis_length * elevationParams.resolution);
        int img_height = static_cast<int>(elevationParams.height * elevationParams.resolution);
        
        orthoRenderWindow->SetSize(img_width, img_height);
        orthoRenderWindow->Render();
        
        // Export image
        writeImage(orthoRenderWindow, fileName);
    }
    
    void exportPlanOrthophoto(const QString& fileName) {
        // Get parameters
        planParams.corner_x = plan_corner_x->value();
        planParams.corner_y = plan_corner_y->value();
        planParams.cutting_height_z = plan_cutting_z->value();
        planParams.width = plan_width->value();
        planParams.height = plan_height->value();
        planParams.depth = plan_depth->value();
        planParams.depth_up = (plan_depth_direction->currentIndex() == 0);
        planParams.resolution = plan_resolution->value();
        
        // Configure VTK for orthographic rendering
        vtkSmartPointer<vtkGenericOpenGLRenderWindow> orthoRenderWindow = 
            vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
        vtkSmartPointer<vtkRenderer> orthoRenderer = vtkSmartPointer<vtkRenderer>::New();
        orthoRenderWindow->AddRenderer(orthoRenderer);
        orthoRenderer->SetBackground(1.0, 1.0, 1.0);
        
        // Add data (mesh or point cloud)
        if (!mesh.cloud.data.empty()) {
            vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
            convertPCLMeshToVTK(mesh, polyData);
            
            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(polyData);
            
            vtkSmartPointer<vtkActor> orthoActor = vtkSmartPointer<vtkActor>::New();
            orthoActor->SetMapper(mapper);
            orthoRenderer->AddActor(orthoActor);
        } else if (cloud && !cloud->empty()) {
            addCloudToRenderer(orthoRenderer, cloud);
        }
        
        // Configure camera for plan view
        vtkSmartPointer<vtkCamera> camera = orthoRenderer->GetActiveCamera();
        
        double center_x = planParams.corner_x + planParams.width / 2.0;
        double center_y = planParams.corner_y + planParams.height / 2.0;
        double center_z = planParams.cutting_height_z;
        
        double cam_dist = planParams.depth * 2.0;
        double cam_z = planParams.depth_up ? 
            (center_z + cam_dist) : (center_z - cam_dist);
        
        camera->SetPosition(center_x, center_y, cam_z);
        camera->SetFocalPoint(center_x, center_y, center_z);
        camera->SetViewUp(0, 1, 0);
        camera->ParallelProjectionOn();
        
        // Calculate image size based on resolution
        int img_width = static_cast<int>(planParams.width * planParams.resolution);
        int img_height = static_cast<int>(planParams.height * planParams.resolution);
        
        orthoRenderWindow->SetSize(img_width, img_height);
        orthoRenderWindow->Render();
        
        // Export image
        writeImage(orthoRenderWindow, fileName);
    }
    
    void exportLegacyOrthophoto(const QString& fileName) {
        // Legacy system using XYZ corners
        orthoCorners = {
            xMinSpinBox->value(), yMinSpinBox->value(), zMinSpinBox->value(),
            xMaxSpinBox->value(), yMaxSpinBox->value(), zMaxSpinBox->value()
        };
        
        // Configure VTK for rendering
        vtkSmartPointer<vtkGenericOpenGLRenderWindow> orthoRenderWindow = 
            vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
        vtkSmartPointer<vtkRenderer> orthoRenderer = vtkSmartPointer<vtkRenderer>::New();
        orthoRenderWindow->AddRenderer(orthoRenderer);
        orthoRenderer->SetBackground(1.0, 1.0, 1.0);
        
        if (!mesh.cloud.data.empty()) {
            vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
            convertPCLMeshToVTK(mesh, polyData);
            
            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(polyData);
            
            vtkSmartPointer<vtkActor> orthoActor = vtkSmartPointer<vtkActor>::New();
            orthoActor->SetMapper(mapper);
            orthoRenderer->AddActor(orthoActor);
        } else if (cloud && !cloud->empty()) {
            addCloudToRenderer(orthoRenderer, cloud);
        }
        
        orthoRenderWindow->SetSize(2048, 2048);
        orthoRenderWindow->Render();
        
        writeImage(orthoRenderWindow, fileName);
    }
    
    void addCloudToRenderer(vtkSmartPointer<vtkRenderer> renderer, 
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud) {
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3);
        colors->SetName("Colors");

        for (const auto& point : pointCloud->points) {
            points->InsertNextPoint(point.x, point.y, point.z);
            unsigned char color[3] = { 
                static_cast<unsigned char>(point.r),
                static_cast<unsigned char>(point.g),
                static_cast<unsigned char>(point.b) 
            };
            colors->InsertNextTuple3(color[0], color[1], color[2]);
        }

        vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
        polyData->SetPoints(points);
        polyData->GetPointData()->SetScalars(colors);

        vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = 
            vtkSmartPointer<vtkVertexGlyphFilter>::New();
        vertexFilter->SetInputData(polyData);
        vertexFilter->Update();

        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(vertexFilter->GetOutputPort());

        vtkSmartPointer<vtkActor> cloudActor = vtkSmartPointer<vtkActor>::New();
        cloudActor->SetMapper(mapper);
        cloudActor->GetProperty()->SetPointSize(2);
        renderer->AddActor(cloudActor);
    }
    
    void writeImage(vtkSmartPointer<vtkGenericOpenGLRenderWindow> window, 
                   const QString& fileName) {
        vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = 
            vtkSmartPointer<vtkWindowToImageFilter>::New();
        windowToImageFilter->SetInput(window);
        windowToImageFilter->Update();

        if (fileName.endsWith(".png")) {
            vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
            writer->SetFileName(fileName.toStdString().c_str());
            writer->SetInputConnection(windowToImageFilter->GetOutputPort());
            writer->Write();
        } else if (fileName.endsWith(".jpg")) {
            vtkSmartPointer<vtkJPEGWriter> writer = vtkSmartPointer<vtkJPEGWriter>::New();
            writer->SetFileName(fileName.toStdString().c_str());
            writer->SetInputConnection(windowToImageFilter->GetOutputPort());
            writer->Write();
        } else if (fileName.endsWith(".tif") || fileName.endsWith(".tiff")) {
            vtkSmartPointer<vtkTIFFWriter> writer = vtkSmartPointer<vtkTIFFWriter>::New();
            writer->SetFileName(fileName.toStdString().c_str());
            writer->SetInputConnection(windowToImageFilter->GetOutputPort());
            writer->Write();
        }
    }

    // 5. Visualize point cloud or mesh
    void visualize() {
        if ((!cloud || cloud->empty()) && mesh.cloud.data.empty()) {
            QMessageBox::warning(this, "Erreur", "Aucune donnée à visualiser!");
            return;
        }

        // Create VTK render window
        renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
        renderer = vtkSmartPointer<vtkRenderer>::New();
        renderWindow->AddRenderer(renderer);

        if (!mesh.cloud.data.empty()) {
            // Visualize mesh
            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

            // Convert PCL mesh to vtkPolyData
            convertPCLMeshToVTK(mesh, polyData);
            mapper->SetInputData(polyData);
            actor = vtkSmartPointer<vtkActor>::New();
            actor->SetMapper(mapper);
            renderer->AddActor(actor);
        } else if (cloud && !cloud->empty()) {
            // Visualize point cloud
            addCloudToRenderer(renderer, cloud);
        }

        renderWindow->SetSize(1024, 768);
        renderWindow->Render();
        vtkSmartPointer<vtkRenderWindowInteractor> interactor = 
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
        interactor->SetRenderWindow(renderWindow);
        interactor->Start();
    }
    
    // Batch processing functions
    void addToBatch() {
        if (mesh.cloud.data.empty() && (!cloud || cloud->empty())) {
            QMessageBox::warning(this, "Erreur", "Aucun mesh ou nuage de points généré!");
            return;
        }
        
        // Ask for configuration name
        bool ok;
        QString name = QInputDialog::getText(this, "Nom de la configuration",
                                            "Nom de l'orthophoto:",
                                            QLineEdit::Normal,
                                            "Orthophoto " + QString::number(batchConfigs.size() + 1),
                                            &ok);
        if (!ok || name.isEmpty()) {
            return;
        }
        
        // Ask for output file path
        QString fileName = QFileDialog::getSaveFileName(this,
            "Chemin de sortie pour " + name,
            name + ".png",
            "PNG Files (*.png);;JPEG Files (*.jpg);;TIFF Files (*.tif)");
        
        if (fileName.isEmpty()) {
            return;
        }
        
        // Determine current mode
        int currentTab = orthoModeTab->currentIndex();
        
        OrthoConfig config;
        config.name = name;
        config.outputPath = fileName;
        
        if (currentTab == 0) {
            // Elevation mode
            config.mode = OrthoMode::Elevation;
            config.elevationParams.point1_x = elev_p1x->value();
            config.elevationParams.point1_y = elev_p1y->value();
            config.elevationParams.point2_x = elev_p2x->value();
            config.elevationParams.point2_y = elev_p2y->value();
            config.elevationParams.start_z = elev_start_z->value();
            config.elevationParams.height = elev_height->value();
            config.elevationParams.depth = elev_depth->value();
            config.elevationParams.depth_forward = (elev_depth_direction->currentIndex() == 0);
            config.elevationParams.resolution = elev_resolution->value();
        } else if (currentTab == 1) {
            // Plan mode
            config.mode = OrthoMode::Plan;
            config.planParams.corner_x = plan_corner_x->value();
            config.planParams.corner_y = plan_corner_y->value();
            config.planParams.cutting_height_z = plan_cutting_z->value();
            config.planParams.width = plan_width->value();
            config.planParams.height = plan_height->value();
            config.planParams.depth = plan_depth->value();
            config.planParams.depth_up = (plan_depth_direction->currentIndex() == 0);
            config.planParams.resolution = plan_resolution->value();
        } else if (currentTab == 2) {
            // Legacy mode
            config.mode = OrthoMode::Legacy;
            config.legacyCorners.x_min = xMinSpinBox->value();
            config.legacyCorners.y_min = yMinSpinBox->value();
            config.legacyCorners.z_min = zMinSpinBox->value();
            config.legacyCorners.x_max = xMaxSpinBox->value();
            config.legacyCorners.y_max = yMaxSpinBox->value();
            config.legacyCorners.z_max = zMaxSpinBox->value();
        } else {
            return; // Batch tab selected, ignore
        }
        
        batchConfigs.append(config);
        updateBatchList();
        
        QMessageBox::information(this, "Succès", 
            "Configuration '" + name + "' ajoutée au batch (" + 
            QString::number(batchConfigs.size()) + " configuration(s))");
    }
    
    void removeFromBatch() {
        int currentRow = batchListWidget->currentRow();
        if (currentRow < 0) {
            QMessageBox::warning(this, "Erreur", "Aucune configuration sélectionnée!");
            return;
        }
        
        batchConfigs.removeAt(currentRow);
        updateBatchList();
    }
    
    void clearBatch() {
        if (batchConfigs.isEmpty()) {
            return;
        }
        
        QMessageBox::StandardButton reply = QMessageBox::question(this,
            "Confirmer",
            "Voulez-vous vraiment vider le batch (" + QString::number(batchConfigs.size()) + " configuration(s))?",
            QMessageBox::Yes | QMessageBox::No);
        
        if (reply == QMessageBox::Yes) {
            batchConfigs.clear();
            updateBatchList();
        }
    }
    
    void updateBatchList() {
        batchListWidget->clear();
        for (const auto& config : batchConfigs) {
            QString modeStr;
            if (config.mode == OrthoMode::Elevation) {
                modeStr = "Élévation";
            } else if (config.mode == OrthoMode::Plan) {
                modeStr = "Plan";
            } else {
                modeStr = "XYZ";
            }
            batchListWidget->addItem(config.name + " [" + modeStr + "] -> " + config.outputPath);
        }
    }
    
    void processBatch() {
        if (batchConfigs.isEmpty()) {
            QMessageBox::warning(this, "Erreur", "Aucune configuration dans le batch!");
            return;
        }
        
        if (mesh.cloud.data.empty() && (!cloud || cloud->empty())) {
            QMessageBox::warning(this, "Erreur", "Aucun mesh ou nuage de points généré!");
            return;
        }
        
        statusLabel->setText("Traitement du batch en cours...");
        progressBar->setValue(0);
        qApp->processEvents();
        
        int totalConfigs = batchConfigs.size();
        int successCount = 0;
        int failCount = 0;
        
        for (int i = 0; i < totalConfigs; ++i) {
            const OrthoConfig& config = batchConfigs[i];
            
            try {
                statusLabel->setText("Traitement: " + config.name + " (" + 
                                   QString::number(i + 1) + "/" + QString::number(totalConfigs) + ")");
                qApp->processEvents();
                
                if (config.mode == OrthoMode::Elevation) {
                    elevationParams = config.elevationParams;
                    exportElevationOrthophoto(config.outputPath);
                } else if (config.mode == OrthoMode::Plan) {
                    planParams = config.planParams;
                    exportPlanOrthophoto(config.outputPath);
                } else {
                    orthoCorners = config.legacyCorners;
                    exportLegacyOrthophoto(config.outputPath);
                }
                
                successCount++;
            } catch (const std::exception& e) {
                failCount++;
                QMessageBox::warning(this, "Erreur", 
                    "Erreur lors du traitement de '" + config.name + "': " + QString(e.what()));
            }
            
            progressBar->setValue((i + 1) * 100 / totalConfigs);
            qApp->processEvents();
        }
        
        statusLabel->setText("Batch terminé: " + QString::number(successCount) + 
                           " succès, " + QString::number(failCount) + " échecs.");
        progressBar->setValue(100);
        
        QMessageBox::information(this, "Batch terminé",
            "Traitement terminé!\n\nSuccès: " + QString::number(successCount) +
            "\nÉchecs: " + QString::number(failCount));
    }
    
    // Work file management functions
    void newWorkFile() {
        QMessageBox::StandardButton reply = QMessageBox::question(this,
            "Nouveau fichier",
            "Créer un nouveau fichier effacera toutes les configurations actuelles. Continuer?",
            QMessageBox::Yes | QMessageBox::No);
        
        if (reply == QMessageBox::Yes) {
            batchConfigs.clear();
            updateBatchList();
            currentWorkFile.clear();
            setWindowTitle("Orthophoto & Mesh Tool 2.0");
        }
    }
    
    void openWorkFile() {
        QString fileName = QFileDialog::getOpenFileName(this,
            "Ouvrir fichier de travail",
            "",
            "Fichiers de travail Orthophoto (*.orth);;Tous les fichiers (*.*)");
        
        if (fileName.isEmpty()) {
            return;
        }
        
        QFile file(fileName);
        if (!file.open(QIODevice::ReadOnly)) {
            QMessageBox::critical(this, "Erreur", 
                "Impossible d'ouvrir le fichier: " + fileName);
            return;
        }
        
        QByteArray data = file.readAll();
        file.close();
        
        QJsonDocument doc = QJsonDocument::fromJson(data);
        if (doc.isNull() || !doc.isObject()) {
            QMessageBox::critical(this, "Erreur", "Format de fichier invalide!");
            return;
        }
        
        QJsonObject root = doc.object();
        
        // Load point cloud file path
        if (root.contains("pointCloudFile")) {
            currentFile = root["pointCloudFile"].toString();
            fileLabel->setText("Fichier: " + currentFile);
        }
        
        // Load batch configurations
        batchConfigs.clear();
        if (root.contains("batchConfigs") && root["batchConfigs"].isArray()) {
            QJsonArray configsArray = root["batchConfigs"].toArray();
            
            for (const QJsonValue& val : configsArray) {
                if (!val.isObject()) continue;
                
                QJsonObject configObj = val.toObject();
                OrthoConfig config;
                
                config.name = configObj["name"].toString();
                config.outputPath = configObj["outputPath"].toString();
                
                QString modeStr = configObj["mode"].toString();
                if (modeStr == "Elevation") {
                    config.mode = OrthoMode::Elevation;
                    QJsonObject params = configObj["parameters"].toObject();
                    config.elevationParams.point1_x = params["point1_x"].toDouble();
                    config.elevationParams.point1_y = params["point1_y"].toDouble();
                    config.elevationParams.point2_x = params["point2_x"].toDouble();
                    config.elevationParams.point2_y = params["point2_y"].toDouble();
                    config.elevationParams.start_z = params["start_z"].toDouble();
                    config.elevationParams.height = params["height"].toDouble();
                    config.elevationParams.depth = params["depth"].toDouble();
                    config.elevationParams.depth_forward = params["depth_forward"].toBool();
                    config.elevationParams.resolution = params["resolution"].toDouble();
                } else if (modeStr == "Plan") {
                    config.mode = OrthoMode::Plan;
                    QJsonObject params = configObj["parameters"].toObject();
                    config.planParams.corner_x = params["corner_x"].toDouble();
                    config.planParams.corner_y = params["corner_y"].toDouble();
                    config.planParams.cutting_height_z = params["cutting_height_z"].toDouble();
                    config.planParams.width = params["width"].toDouble();
                    config.planParams.height = params["height"].toDouble();
                    config.planParams.depth = params["depth"].toDouble();
                    config.planParams.depth_up = params["depth_up"].toBool();
                    config.planParams.resolution = params["resolution"].toDouble();
                } else {
                    config.mode = OrthoMode::Legacy;
                    QJsonObject params = configObj["parameters"].toObject();
                    config.legacyCorners.x_min = params["x_min"].toDouble();
                    config.legacyCorners.y_min = params["y_min"].toDouble();
                    config.legacyCorners.z_min = params["z_min"].toDouble();
                    config.legacyCorners.x_max = params["x_max"].toDouble();
                    config.legacyCorners.y_max = params["y_max"].toDouble();
                    config.legacyCorners.z_max = params["z_max"].toDouble();
                }
                
                batchConfigs.append(config);
            }
        }
        
        updateBatchList();
        currentWorkFile = fileName;
        setWindowTitle("Orthophoto & Mesh Tool 2.0 - " + QFileInfo(fileName).fileName());
        
        QMessageBox::information(this, "Succès", 
            "Fichier chargé: " + QString::number(batchConfigs.size()) + " configuration(s)");
    }
    
    void saveWorkFile() {
        if (currentWorkFile.isEmpty()) {
            saveWorkFileAs();
        } else {
            saveWorkFileToPath(currentWorkFile);
        }
    }
    
    void saveWorkFileAs() {
        QString fileName = QFileDialog::getSaveFileName(this,
            "Enregistrer fichier de travail",
            "",
            "Fichiers de travail Orthophoto (*.orth)");
        
        if (!fileName.isEmpty()) {
            if (!fileName.endsWith(".orth")) {
                fileName += ".orth";
            }
            saveWorkFileToPath(fileName);
        }
    }
    
    void saveWorkFileToPath(const QString& filePath) {
        QJsonObject root;
        
        // Save point cloud file path
        root["pointCloudFile"] = currentFile;
        
        // Save batch configurations
        QJsonArray configsArray;
        for (const OrthoConfig& config : batchConfigs) {
            QJsonObject configObj;
            configObj["name"] = config.name;
            configObj["outputPath"] = config.outputPath;
            
            if (config.mode == OrthoMode::Elevation) {
                configObj["mode"] = "Elevation";
                QJsonObject params;
                params["point1_x"] = config.elevationParams.point1_x;
                params["point1_y"] = config.elevationParams.point1_y;
                params["point2_x"] = config.elevationParams.point2_x;
                params["point2_y"] = config.elevationParams.point2_y;
                params["start_z"] = config.elevationParams.start_z;
                params["height"] = config.elevationParams.height;
                params["depth"] = config.elevationParams.depth;
                params["depth_forward"] = config.elevationParams.depth_forward;
                params["resolution"] = config.elevationParams.resolution;
                configObj["parameters"] = params;
            } else if (config.mode == OrthoMode::Plan) {
                configObj["mode"] = "Plan";
                QJsonObject params;
                params["corner_x"] = config.planParams.corner_x;
                params["corner_y"] = config.planParams.corner_y;
                params["cutting_height_z"] = config.planParams.cutting_height_z;
                params["width"] = config.planParams.width;
                params["height"] = config.planParams.height;
                params["depth"] = config.planParams.depth;
                params["depth_up"] = config.planParams.depth_up;
                params["resolution"] = config.planParams.resolution;
                configObj["parameters"] = params;
            } else {
                configObj["mode"] = "Legacy";
                QJsonObject params;
                params["x_min"] = config.legacyCorners.x_min;
                params["y_min"] = config.legacyCorners.y_min;
                params["z_min"] = config.legacyCorners.z_min;
                params["x_max"] = config.legacyCorners.x_max;
                params["y_max"] = config.legacyCorners.y_max;
                params["z_max"] = config.legacyCorners.z_max;
                configObj["parameters"] = params;
            }
            
            configsArray.append(configObj);
        }
        root["batchConfigs"] = configsArray;
        
        QJsonDocument doc(root);
        
        QFile file(filePath);
        if (!file.open(QIODevice::WriteOnly)) {
            QMessageBox::critical(this, "Erreur", 
                "Impossible d'enregistrer le fichier: " + filePath);
            return;
        }
        
        file.write(doc.toJson());
        file.close();
        
        currentWorkFile = filePath;
        setWindowTitle("Orthophoto & Mesh Tool 2.0 - " + QFileInfo(filePath).fileName());
        statusBar()->showMessage("Fichier enregistré: " + filePath);
    }
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    OrthophotoGUI window;
    window.show();
    return app.exec();
}

#include "orthophoto_gui.moc"
