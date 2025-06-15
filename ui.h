#ifndef UI_H
#define UI_H

#include <QObject>
#include <QLabel>
#include <QPushButton>
#include <QMenu>
#include <QAction>
#include <QActionGroup>

class MainWindow;

class UI : public QObject
{
    Q_OBJECT

public:
    explicit UI(MainWindow* parent);
    ~UI();

    void setupUI();
    void updateStatusLabel();
    void setRedLinesVisibility(bool visible);
    void setTargetButtonText(bool targetMode);
    void setObstacleButtonText(bool obstacleMode);

private slots:
    void onToggleLinesClicked();
    void onOptimizeClicked();
    void onClearAllClicked();
    void onSetTargetClicked();
    void onCreateObstacleClicked();
    void onShowTracingModeMenu();
    void onSetAddMode();
    void onSetReplaceMode();

private:
    void createStatusLabel();
    void createControlButtons();
    void createTracingModeMenu();
    void connectSignals();

    MainWindow* m_mainWindow;

    // UI элементы
    QLabel* m_statusLabel;
    QPushButton* m_toggleLinesButton;
    QPushButton* m_optimizeButton;
    QPushButton* m_clearAllButton;
    QPushButton* m_setTargetButton;
    QPushButton* m_createObstacleButton;
    QPushButton* m_tracingModeButton;

    QMenu* m_tracingModeMenu;
    QAction* m_addModeAction;
    QAction* m_replaceModeAction;
    QActionGroup* m_modeActionGroup;
};

#endif // UIMANAGER_H
