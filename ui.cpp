#include "ui.h"
#include "mainwindow.h"
#include <QApplication>

UI::UI(MainWindow* parent)
    : QObject(parent)
    , m_mainWindow(parent)
    , m_statusLabel(nullptr)
    , m_toggleLinesButton(nullptr)
    , m_optimizeButton(nullptr)
    , m_clearAllButton(nullptr)
    , m_setTargetButton(nullptr)
    , m_createObstacleButton(nullptr)
    , m_tracingModeButton(nullptr)
    , m_tracingModeMenu(nullptr)
    , m_addModeAction(nullptr)
    , m_replaceModeAction(nullptr)
    , m_modeActionGroup(nullptr)
{
}

UI::~UI()
{
    // Деструктор - элементы UI удалятся автоматически через parent
}

void UI::setupUI()
{
    createStatusLabel();
    createControlButtons();
    createTracingModeMenu();
    connectSignals();
}

void UI::createStatusLabel()
{
    m_statusLabel = new QLabel("Режим: Добавление трасс", m_mainWindow);
    m_statusLabel->setGeometry(20, 20, 250, 30);
    m_statusLabel->setStyleSheet(
        "QLabel {"
        "    background-color: #f0f0f0;"
        "    border: 2px solid #cccccc;"
        "    border-radius: 5px;"
        "    padding: 5px;"
        "    font-weight: bold;"
        "    color: #333333;"
        "}"
        );
}

void UI::createControlButtons()
{
    m_toggleLinesButton = new QPushButton("Скрыть красные лучи", m_mainWindow);
    m_toggleLinesButton->setGeometry(300, 20, 180, 30);

    m_optimizeButton = new QPushButton("Оптимизировать трассу", m_mainWindow);
    m_optimizeButton->setGeometry(300, 60, 180, 30);

    m_clearAllButton = new QPushButton("Очистить все лучи", m_mainWindow);
    m_clearAllButton->setGeometry(500, 20, 180, 30);

    m_setTargetButton = new QPushButton("Задать конечную точку", m_mainWindow);
    m_setTargetButton->setGeometry(700, 20, 180, 30);

    m_createObstacleButton = new QPushButton("Создать препятствие", m_mainWindow);
    m_createObstacleButton->setGeometry(700, 60, 180, 30);

    m_tracingModeButton = new QPushButton("Режимы трассировки", m_mainWindow);
    m_tracingModeButton->setGeometry(500, 60, 180, 30);
}

void UI::createTracingModeMenu()
{
    m_tracingModeMenu = new QMenu("Режимы трассировки", m_mainWindow);

    m_addModeAction = new QAction("Режим добавления", this);
    m_addModeAction->setCheckable(true);
    m_addModeAction->setChecked(true);

    m_replaceModeAction = new QAction("Режим замены", this);
    m_replaceModeAction->setCheckable(true);
    m_replaceModeAction->setChecked(false);

    m_modeActionGroup = new QActionGroup(this);
    m_modeActionGroup->addAction(m_addModeAction);
    m_modeActionGroup->addAction(m_replaceModeAction);

    m_tracingModeMenu->addAction(m_addModeAction);
    m_tracingModeMenu->addAction(m_replaceModeAction);
}

void UI::connectSignals()
{
    connect(m_toggleLinesButton, &QPushButton::clicked, this, &UI::onToggleLinesClicked);
    connect(m_optimizeButton, &QPushButton::clicked, this, &UI::onOptimizeClicked);
    connect(m_clearAllButton, &QPushButton::clicked, this, &UI::onClearAllClicked);
    connect(m_setTargetButton, &QPushButton::clicked, this, &UI::onSetTargetClicked);
    connect(m_createObstacleButton, &QPushButton::clicked, this, &UI::onCreateObstacleClicked);
    connect(m_tracingModeButton, &QPushButton::clicked, this, &UI::onShowTracingModeMenu);
    connect(m_addModeAction, &QAction::triggered, this, &UI::onSetAddMode);
    connect(m_replaceModeAction, &QAction::triggered, this, &UI::onSetReplaceMode);
}

void UI::updateStatusLabel()
{
    QString statusText;
    if (m_mainWindow->isInTargetMode()) {
        statusText = "Режим: Задание конечной точки";
    } else if (m_mainWindow->isInObstacleMode()) {
        statusText = "Режим: Создание препятствий";
    } else {
        statusText = m_mainWindow->isInAddMode() ? "Режим: Добавление трасс" : "Режим: Замена трасс";
    }
    m_statusLabel->setText(statusText);
}

void UI::setRedLinesVisibility(bool visible)
{
    m_toggleLinesButton->setText(visible ? "Скрыть красные лучи" : "Показать красные лучи");
}

// Слоты для обработки нажатий кнопок
void UI::onToggleLinesClicked()
{
    m_mainWindow->toggleLinesVisibility();
}

void UI::onOptimizeClicked()
{
    m_mainWindow->performOptimization();
}

void UI::onClearAllClicked()
{
    m_mainWindow->clearAllRays();
}

void UI::onSetTargetClicked()
{
    m_mainWindow->toggleTargetMode();
}

void UI::onCreateObstacleClicked()
{
    m_mainWindow->toggleObstacleMode();
}

void UI::onShowTracingModeMenu()
{
    QPoint menuPos = m_tracingModeButton->mapToGlobal(QPoint(0, m_tracingModeButton->height()));
    m_tracingModeMenu->exec(menuPos);
}

void UI::onSetAddMode()
{
    m_mainWindow->setAddMode();
    updateStatusLabel();
}

void UI::onSetReplaceMode()
{
    m_mainWindow->setReplaceMode();
    updateStatusLabel();
}

void UI::setTargetButtonText(bool targetMode)
{
    if (m_setTargetButton) {
        m_setTargetButton->setText(targetMode ? "Отменить задание цели" : "Задать конечную точку");
    }
}

void UI::setObstacleButtonText(bool obstacleMode)
{
    if (m_createObstacleButton) {
        m_createObstacleButton->setText(obstacleMode ? "Отменить создание" : "Создать препятствие");
    }
}

