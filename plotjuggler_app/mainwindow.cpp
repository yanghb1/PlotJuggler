/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <functional>
#include <stdio.h>

#include <QApplication>
#include <QActionGroup>
#include <QCheckBox>
#include <QCommandLineParser>
#include <QDebug>
#include <QDesktopServices>
#include <QDomDocument>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFileDialog>
#include <QInputDialog>
#include <QMenu>
#include <QGroupBox>
#include <QMessageBox>
#include <QMimeData>
#include <QMouseEvent>
#include <QPluginLoader>
#include <QPushButton>
#include <QKeySequence>
#include <QScrollBar>
#include <QSettings>
#include <QStringListModel>
#include <QStringRef>
#include <QThread>
#include <QTextStream>
#include <QWindow>
#include <QHeaderView>
#include <QStandardPaths>
#include <QXmlStreamReader>

#include "mainwindow.h"
#include "curvelist_panel.h"
#include "tabbedplotwidget.h"
#include "PlotJuggler/plotdata.h"
#include "transforms/function_editor.h"
#include "transforms/lua_custom_function.h"
#include "utils.h"
#include "stylesheet.h"
#include "dummy_data.h"
#include "PlotJuggler/svg_util.h"
#include "PlotJuggler/reactive_function.h"
#include "multifile_prefix.h"

#include "ui_aboutdialog.h"
#include "ui_support_dialog.h"
#include "preferences_dialog.h"
#include "nlohmann_parsers.h"
#include "cheatsheet/cheatsheet_dialog.h"
#include "colormap_editor.h"

#ifdef COMPILED_WITH_CATKIN

#endif
#ifdef COMPILED_WITH_AMENT
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

MainWindow::MainWindow(const QCommandLineParser& commandline_parser, QWidget* parent)
  : QMainWindow(parent)
  , ui(new Ui::MainWindow)
  , _undo_shortcut(QKeySequence(Qt::CTRL + Qt::Key_Z), this)
  , _redo_shortcut(QKeySequence(Qt::CTRL + Qt::SHIFT + Qt::Key_Z), this)
  , _fullscreen_shortcut(Qt::Key_F10, this)
  , _streaming_shortcut(QKeySequence(Qt::CTRL + Qt::Key_Space), this)
  , _playback_shotcut(Qt::Key_Space, this)
  , _minimized(false)
  , _active_streamer_plugin(nullptr)
  , _disable_undo_logging(false)
  , _tracker_time(0)
  , _tracker_param(CurveTracker::VALUE)
  , _labels_status(LabelStatus::RIGHT)
  , _recent_data_files(new QMenu())
  , _recent_layout_files(new QMenu())
{
  QLocale::setDefault(QLocale::c());  // set as default
  setAcceptDrops(true);

  _test_option = commandline_parser.isSet("test");
  _autostart_publishers = commandline_parser.isSet("publish");

  if (commandline_parser.isSet("enabled_plugins"))
  {
    _enabled_plugins =
        commandline_parser.value("enabled_plugins").split(";", PJ::SkipEmptyParts);
    // Treat the command-line parameter  '--enabled_plugins *' to mean all plugings are
    // enabled
    if ((_enabled_plugins.size() == 1) && (_enabled_plugins.contains("*")))
    {
      _enabled_plugins.clear();
    }
  }
  if (commandline_parser.isSet("disabled_plugins"))
  {
    _disabled_plugins =
        commandline_parser.value("disabled_plugins").split(";", PJ::SkipEmptyParts);
  }

  _curvelist_widget = new CurveListPanel(_mapped_plot_data, _transform_functions, this);

  ui->setupUi(this);

  // setupUi() sets the windowTitle so the skin-based setting must be done after
  _skin_path = "://resources/skin";
  if (commandline_parser.isSet("skin_path"))
  {
    QDir path(commandline_parser.value("skin_path"));
    if (path.exists())
    {
      _skin_path = path.absolutePath();
    }
  }
  if (commandline_parser.isSet("window_title"))
  {
    setWindowTitle(commandline_parser.value("window_title"));
  }
  else
  {
    QFile fileTitle(_skin_path + "/mainwindow_title.txt");
    if (fileTitle.open(QIODevice::ReadOnly))
    {
      QString title = fileTitle.readAll().trimmed();
      setWindowTitle(title);
    }
  }

  QSettings settings;

  ui->playbackLoop->setText("");
  ui->buttonZoomOut->setText("");
  ui->buttonPlay->setText("");
  ui->buttonUseDateTime->setText("");
  ui->buttonActivateGrid->setText("");
  ui->buttonRatio->setText("");
  ui->buttonLink->setText("");
  ui->buttonTimeTracker->setText("");
  ui->buttonLoadDatafile->setText("");
  ui->buttonRemoveTimeOffset->setText("");
  ui->buttonLegend->setText("");

  ui->widgetStatusBar->setHidden(true);

  if (commandline_parser.isSet("buffer_size"))
  {
    int buffer_size = std::max(10, commandline_parser.value("buffer_size").toInt());
    ui->streamingSpinBox->setMaximum(buffer_size);
  }

  _animated_streaming_movie = new QMovie(":/resources/animated_radio.gif");
  _animated_streaming_movie->setScaledSize(ui->labelStreamingAnimation->size());
  _animated_streaming_movie->jumpToFrame(0);

  _animated_streaming_timer = new QTimer();
  _animated_streaming_timer->setSingleShot(true);

  connect(_animated_streaming_timer, &QTimer::timeout, this, [this]() {
    _animated_streaming_movie->stop();
    _animated_streaming_movie->jumpToFrame(0);
  });

  _tracker_delay.connectCallback([this]() {
    updatedDisplayTime();
    onUpdateLeftTableValues();
  });

  ui->labelStreamingAnimation->setMovie(_animated_streaming_movie);
  ui->labelStreamingAnimation->setHidden(true);

  connect(this, &MainWindow::stylesheetChanged, this, &MainWindow::on_stylesheetChanged);

  connect(this, &MainWindow::stylesheetChanged, _curvelist_widget,
          &CurveListPanel::on_stylesheetChanged);

  connect(_curvelist_widget, &CurveListPanel::hiddenItemsChanged, this,
          &MainWindow::onUpdateLeftTableValues);

  connect(_curvelist_widget, &CurveListPanel::deleteCurves, this,
          &MainWindow::onDeleteMultipleCurves);

  connect(_curvelist_widget, &CurveListPanel::createMathPlot, this,
          &MainWindow::onAddCustomPlot);

  connect(_curvelist_widget, &CurveListPanel::editMathPlot, this,
          &MainWindow::onEditCustomPlot);

  connect(_curvelist_widget, &CurveListPanel::refreshMathPlot, this,
          &MainWindow::onRefreshCustomPlot);

  connect(ui->timeSlider, &RealSlider::realValueChanged, this,
          &MainWindow::onTimeSlider_valueChanged);

  connect(ui->playbackRate, &QDoubleSpinBox::editingFinished, this,
          [this]() { ui->playbackRate->clearFocus(); });

  connect(ui->playbackStep, &QDoubleSpinBox::editingFinished, this,
          [this]() { ui->playbackStep->clearFocus(); });

  connect(_curvelist_widget, &CurveListPanel::requestDeleteAll, this, [this](int option) {
    if (option == 1)
    {
      deleteAllData();
    }
    else if (option == 2)
    {
      on_actionClearBuffer_triggered();
    }
  });

  _main_tabbed_widget =
      new TabbedPlotWidget("Main Window", this, _mapped_plot_data, this);

  connect(this, &MainWindow::stylesheetChanged, _main_tabbed_widget,
          &TabbedPlotWidget::on_stylesheetChanged);

  ui->plottingLayout->insertWidget(0, _main_tabbed_widget, 1);
  ui->leftLayout->addWidget(_curvelist_widget, 1);

  ui->mainSplitter->setCollapsible(0, true);
  ui->mainSplitter->setStretchFactor(0, 2);
  ui->mainSplitter->setStretchFactor(1, 6);

  ui->layoutTimescale->removeWidget(ui->widgetButtons);
  _main_tabbed_widget->tabWidget()->setCornerWidget(ui->widgetButtons);

  connect(ui->mainSplitter, SIGNAL(splitterMoved(int, int)),
          SLOT(on_splitterMoved(int, int)));

  initializeActions();

  LoadColorMapFromSettings();

  //------------ Load plugins -------------
  auto plugin_extra_folders =
      commandline_parser.value("plugin_folders").split(";", PJ::SkipEmptyParts);

  _default_streamer = commandline_parser.value("start_streamer");

  loadAllPlugins(plugin_extra_folders);

  //------------------------------------

  _undo_timer.start();

  // save initial state
  onUndoableChange();

  _replot_timer = new QTimer(this);
  connect(_replot_timer, &QTimer::timeout, this,
          [this]() { updateDataAndReplot(false); });

  _publish_timer = new QTimer(this);
  _publish_timer->setInterval(20);
  connect(_publish_timer, &QTimer::timeout, this, &MainWindow::onPlaybackLoop);

  ui->menuFile->setToolTipsVisible(true);

  this->setMenuBar(ui->menuBar);
  ui->menuBar->setNativeMenuBar(false);

  if (_test_option)
  {
    buildDummyData();
  }

  bool file_loaded = false;
  if (commandline_parser.isSet("datafile"))
  {
    QStringList datafiles = commandline_parser.values("datafile");
    file_loaded = loadDataFromFiles(datafiles);
  }
  if (commandline_parser.isSet("layout"))
  {
    loadLayoutFromFile(commandline_parser.value("layout"));
  }

  restoreGeometry(settings.value("MainWindow.geometry").toByteArray());
  restoreState(settings.value("MainWindow.state").toByteArray());

  // qDebug() << "restoreGeometry";

  bool activate_grid = settings.value("MainWindow.activateGrid", false).toBool();
  ui->buttonActivateGrid->setChecked(activate_grid);

  bool zoom_link_active = settings.value("MainWindow.buttonLink", true).toBool();
  ui->buttonLink->setChecked(zoom_link_active);

  bool ration_active = settings.value("MainWindow.buttonRatio", true).toBool();
  ui->buttonRatio->setChecked(ration_active);

  int streaming_buffer_value =
      settings.value("MainWindow.streamingBufferValue", 5).toInt();
  ui->streamingSpinBox->setValue(streaming_buffer_value);

  bool datetime_display = settings.value("MainWindow.dateTimeDisplay", false).toBool();
  ui->buttonUseDateTime->setChecked(datetime_display);

  bool remove_time_offset = settings.value("MainWindow.removeTimeOffset", true).toBool();
  ui->buttonRemoveTimeOffset->setChecked(remove_time_offset);

  if (settings.value("MainWindow.hiddenFileFrame", false).toBool())
  {
    ui->buttonHideFileFrame->setText("+");
    ui->frameFile->setHidden(true);
  }
  if (settings.value("MainWindow.hiddenStreamingFrame", false).toBool())
  {
    ui->buttonHideStreamingFrame->setText("+");
    ui->frameStreaming->setHidden(true);
  }
  if (settings.value("MainWindow.hiddenPublishersFrame", false).toBool())
  {
    ui->buttonHidePublishersFrame->setText("+");
    ui->framePublishers->setHidden(true);
  }

  //----------------------------------------------------------
  QIcon trackerIconA, trackerIconB, trackerIconC;

  trackerIconA.addFile(QStringLiteral(":/style_light/line_tracker.png"), QSize(36, 36));
  trackerIconB.addFile(QStringLiteral(":/style_light/line_tracker_1.png"), QSize(36, 36));
  trackerIconC.addFile(QStringLiteral(":/style_light/line_tracker_a.png"), QSize(36, 36));

  _tracker_button_icons[CurveTracker::LINE_ONLY] = trackerIconA;
  _tracker_button_icons[CurveTracker::VALUE] = trackerIconB;
  _tracker_button_icons[CurveTracker::VALUE_NAME] = trackerIconC;

  int tracker_setting =
      settings.value("MainWindow.timeTrackerSetting", (int)CurveTracker::VALUE).toInt();
  _tracker_param = static_cast<CurveTracker::Parameter>(tracker_setting);

  ui->buttonTimeTracker->setIcon(_tracker_button_icons[_tracker_param]);

  forEachWidget([&](PlotWidget* plot) { plot->configureTracker(_tracker_param); });

  auto editor_layout = new QVBoxLayout();
  editor_layout->setMargin(0);
  ui->formulaPage->setLayout(editor_layout);
  _function_editor =
      new FunctionEditorWidget(_mapped_plot_data, _transform_functions, this);
  editor_layout->addWidget(_function_editor);

  connect(_function_editor, &FunctionEditorWidget::closed, this,
          [this]() { ui->widgetStack->setCurrentIndex(0); });

  connect(this, &MainWindow::stylesheetChanged, _function_editor,
          &FunctionEditorWidget::on_stylesheetChanged);

  connect(_function_editor, &FunctionEditorWidget::accept, this,
          &MainWindow::onCustomPlotCreated);

  QString theme = settings.value("Preferences::theme", "dark").toString();
  if (theme != "dark")
  {
    theme = "light";
  }

  loadStyleSheet(tr(":/resources/stylesheet_%1.qss").arg(theme));
  // builtin messageParsers
  auto json_parser = std::make_shared<JSON_ParserFactory>();
  _parser_factories.insert({ json_parser->encoding(), json_parser });

  auto cbor_parser = std::make_shared<CBOR_ParserFactory>();
  _parser_factories.insert({ cbor_parser->encoding(), cbor_parser });

  auto bson_parser = std::make_shared<BSON_ParserFactory>();
  _parser_factories.insert({ bson_parser->encoding(), bson_parser });

  auto msgpack = std::make_shared<MessagePack_ParserFactory>();
  _parser_factories.insert({ msgpack->encoding(), msgpack });

  if (!_default_streamer.isEmpty())
  {
    auto index = ui->comboStreaming->findText(_default_streamer);
    if (index != -1)
    {
      ui->comboStreaming->setCurrentIndex(index);
      settings.setValue("MainWindow.previousStreamingPlugin", _default_streamer);
    }
  }

  //-------------------------------------------------------------------
  QLayoutItem *horizontalLayout_11;
  while ((horizontalLayout_11 = ui->horizontalLayout_11->takeAt(0)) != nullptr) {
      // 检查是否是QWidget的指针
      if (horizontalLayout_11->widget()) {
          horizontalLayout_11->widget()->hide();
      }
  }

  ui->frameStreaming->hide();
  ui->widgetLabelPublishers->hide();
  ui->widgetLabelStreaming->hide();
  ui->checkBoxAddPrefixAndMerge->hide();
  ui->line->hide();

  initOpenGLWidget();
}

MainWindow::~MainWindow()
{
  // important: avoid problems with plugins
  _mapped_plot_data.user_defined.clear();

  delete ui;
}

void MainWindow::onUndoableChange()
{
  if (_disable_undo_logging)
    return;

  int elapsed_ms = _undo_timer.restart();

  // overwrite the previous
  if (elapsed_ms < 100)
  {
    if (_undo_states.empty() == false)
      _undo_states.pop_back();
  }

  while (_undo_states.size() >= 100)
    _undo_states.pop_front();
  _undo_states.push_back(xmlSaveState());
  _redo_states.clear();
  // qDebug() << "undo " << _undo_states.size();
}

void MainWindow::onRedoInvoked()
{
  _disable_undo_logging = true;
  if (_redo_states.size() > 0)
  {
    QDomDocument state_document = _redo_states.back();
    while (_undo_states.size() >= 100)
      _undo_states.pop_front();
    _undo_states.push_back(state_document);
    _redo_states.pop_back();

    xmlLoadState(state_document);
  }
  // qDebug() << "undo " << _undo_states.size();
  _disable_undo_logging = false;
}

void MainWindow::onUndoInvoked()
{
  _disable_undo_logging = true;
  if (_undo_states.size() > 1)
  {
    QDomDocument state_document = _undo_states.back();
    while (_redo_states.size() >= 100)
      _redo_states.pop_front();
    _redo_states.push_back(state_document);
    _undo_states.pop_back();
    state_document = _undo_states.back();

    xmlLoadState(state_document);
  }
  // qDebug() << "undo " << _undo_states.size();
  _disable_undo_logging = false;
}

void MainWindow::onUpdateLeftTableValues()
{
  _curvelist_widget->update2ndColumnValues(_tracker_time);
}

void MainWindow::onTrackerMovedFromWidget(QPointF relative_pos)
{
  _tracker_time = relative_pos.x() + _time_offset.get();

  auto prev = ui->timeSlider->blockSignals(true);
  ui->timeSlider->setRealValue(_tracker_time);
  ui->timeSlider->blockSignals(prev);

  onTrackerTimeUpdated(_tracker_time, true);
}

void MainWindow::onTimeSlider_valueChanged(double abs_time)
{
  _tracker_time = abs_time;
  onTrackerTimeUpdated(_tracker_time, true);
}

void MainWindow::onTrackerTimeUpdated(double absolute_time, bool do_replot)
{
  _tracker_delay.triggerSignal(100);

  for (auto& it : _state_publisher)
  {
    it.second->updateState(absolute_time);
  }

  updateReactivePlots();

  forEachWidget([&](PlotWidget* plot) {
    plot->setTrackerPosition(_tracker_time);
    if (do_replot)
    {
      plot->replot();
    }
  });
}

void MainWindow::initializeActions()
{
  _undo_shortcut.setContext(Qt::ApplicationShortcut);
  _redo_shortcut.setContext(Qt::ApplicationShortcut);
  _fullscreen_shortcut.setContext(Qt::ApplicationShortcut);

  connect(&_undo_shortcut, &QShortcut::activated, this, &MainWindow::onUndoInvoked);
  connect(&_redo_shortcut, &QShortcut::activated, this, &MainWindow::onRedoInvoked);
  connect(&_streaming_shortcut, &QShortcut::activated, this,
          &MainWindow::on_streamingToggled);
  connect(&_playback_shotcut, &QShortcut::activated, ui->buttonPlay,
          &QPushButton::toggle);
  connect(&_fullscreen_shortcut, &QShortcut::activated, this,
          &MainWindow::onActionFullscreenTriggered);

  QShortcut* open_menu_shortcut = new QShortcut(QKeySequence(Qt::ALT + Qt::Key_F), this);
  connect(open_menu_shortcut, &QShortcut::activated,
          [this]() { ui->menuFile->exec(ui->menuBar->mapToGlobal(QPoint(0, 25))); });

  QShortcut* open_help_shortcut = new QShortcut(QKeySequence(Qt::ALT + Qt::Key_H), this);
  connect(open_help_shortcut, &QShortcut::activated,
          [this]() { ui->menuHelp->exec(ui->menuBar->mapToGlobal(QPoint(230, 25))); });

  //---------------------------------------------

  QSettings settings;
  updateRecentDataMenu(
      settings.value("MainWindow.recentlyLoadedDatafile").toStringList());
  updateRecentLayoutMenu(
      settings.value("MainWindow.recentlyLoadedLayout").toStringList());
}

void MainWindow::loadAllPlugins(QStringList command_line_plugin_folders)
{
  QSettings settings;
  QStringList loaded;
  QStringList plugin_folders;
  QStringList builtin_folders;

  plugin_folders += command_line_plugin_folders;
  plugin_folders +=
      settings.value("Preferences::plugin_folders", QStringList()).toStringList();

  builtin_folders += QCoreApplication::applicationDirPath();

  try
  {
#ifdef COMPILED_WITH_CATKIN
    builtin_folders += QCoreApplication::applicationDirPath() + "_ros";

    const char* env = std::getenv("CMAKE_PREFIX_PATH");
    if (env)
    {
      QString env_catkin_paths = QString::fromStdString(env);
      env_catkin_paths.replace(";", ":");  // for windows
      auto catkin_paths = env_catkin_paths.split(":");

      for (const auto& path : catkin_paths)
      {
        builtin_folders += path + "/lib/plotjuggler_ros";
      }
    }
#endif
#ifdef COMPILED_WITH_AMENT
    auto ros2_path = QString::fromStdString(ament_index_cpp::get_package_prefix("plotjugg"
                                                                                "ler_"
                                                                                "ros"));
    ros2_path += "/lib/plotjuggler_ros";
    loaded += initializePlugins(ros2_path);
#endif
  }
  catch (...)
  {
    QMessageBox::warning(nullptr, "Missing package [plotjuggler-ros]",
                         "If you just upgraded from PlotJuggler 2.x to 3.x , try "
                         "installing this package:\n\n"
                         "sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros",
                         QMessageBox::Cancel, QMessageBox::Cancel);
  }

  builtin_folders +=
      QStandardPaths::writableLocation(QStandardPaths::GenericDataLocation) + "/PlotJuggl"
                                                                              "er";
  builtin_folders.removeDuplicates();

  plugin_folders += builtin_folders;
  plugin_folders.removeDuplicates();

  for (const auto& folder : plugin_folders)
  {
    loaded += initializePlugins(folder);
  }

  settings.setValue("Preferences::builtin_plugin_folders", builtin_folders);
}

QStringList MainWindow::initializePlugins(QString directory_name)
{
  static std::set<QString> loaded_plugins;
  QStringList loaded_out;

  qDebug() << "Loading compatible plugins from directory: " << directory_name;
  int loaded_count = 0;

  QDir pluginsDir(directory_name);

  for (const QString& filename : pluginsDir.entryList(QDir::Files))
  {
    QFileInfo fileinfo(filename);
    if (fileinfo.suffix() != "so" && fileinfo.suffix() != "dll" &&
        fileinfo.suffix() != "dylib")
    {
      continue;
    }

    if (loaded_plugins.find(filename) != loaded_plugins.end())
    {
      continue;
    }

    QPluginLoader pluginLoader(pluginsDir.absoluteFilePath(filename), this);

    QObject* plugin;
    try
    {
      plugin = pluginLoader.instance();
    }
    catch (std::runtime_error& err)
    {
      qDebug() << QString("%1: skipping, because it threw the following exception: %2")
                      .arg(filename)
                      .arg(err.what());
      continue;
    }
    if (plugin && dynamic_cast<PlotJugglerPlugin*>(plugin))
    {
      auto class_name = pluginLoader.metaData().value("className").toString();
      loaded_out.push_back(class_name);

      DataLoader* loader = qobject_cast<DataLoader*>(plugin);
      StatePublisher* publisher = qobject_cast<StatePublisher*>(plugin);
      DataStreamer* streamer = qobject_cast<DataStreamer*>(plugin);
      ParserFactoryPlugin* message_parser = qobject_cast<ParserFactoryPlugin*>(plugin);
      ToolboxPlugin* toolbox = qobject_cast<ToolboxPlugin*>(plugin);

      QString plugin_name;
      QString plugin_type;
      bool is_debug_plugin = dynamic_cast<PlotJugglerPlugin*>(plugin)->isDebugPlugin();

      if (loader)
      {
        plugin_name = loader->name();
        plugin_type = "DataLoader";
      }
      else if (publisher)
      {
        plugin_name = publisher->name();
        plugin_type = "StatePublisher";
      }
      else if (streamer)
      {
        plugin_name = streamer->name();
        plugin_type = "DataStreamer";
      }
      else if (message_parser)
      {
        plugin_name = message_parser->name();
        plugin_type = "MessageParser";
      }
      else if (toolbox)
      {
        plugin_name = toolbox->name();
        plugin_type = "Toolbox";
      }

      QString message = QString("%1 is a %2 plugin").arg(filename).arg(plugin_type);

      if ((_enabled_plugins.size() > 0) &&
          (_enabled_plugins.contains(fileinfo.baseName()) == false))
      {
        qDebug() << message << " ...skipping, because it is not explicitly enabled";
        continue;
      }
      if ((_disabled_plugins.size() > 0) &&
          (_disabled_plugins.contains(fileinfo.baseName()) == true))
      {
        qDebug() << message << " ...skipping, because it is explicitly disabled";
        continue;
      }
      if (!_test_option && is_debug_plugin)
      {
        qDebug() << message << " ...disabled, unless option -t is used";
        continue;
      }
      if (loaded_plugins.find(plugin_name) != loaded_plugins.end())
      {
        qDebug() << message << " ...skipping, because already loaded";
        continue;
      }

      qDebug() << message;

      loaded_plugins.insert(plugin_name);
      loaded_count++;

      if (loader)
      {
        _data_loader.insert(std::make_pair(plugin_name, loader));
      }
      else if (publisher)
      {
        publisher->setDataMap(&_mapped_plot_data);
        _state_publisher.insert(std::make_pair(plugin_name, publisher));

        ui->layoutPublishers->setColumnStretch(0, 1.0);

        int row = _state_publisher.size() - 1;
        auto label = new QLabel(plugin_name, ui->framePublishers);
        ui->layoutPublishers->addWidget(label, row, 0);

        auto start_checkbox = new QCheckBox(ui->framePublishers);
        ui->layoutPublishers->addWidget(start_checkbox, row, 1);
        start_checkbox->setFocusPolicy(Qt::FocusPolicy::NoFocus);

        connect(start_checkbox, &QCheckBox::toggled, this,
                [=](bool enable) { publisher->setEnabled(enable); });

        connect(publisher, &StatePublisher::closed, start_checkbox,
                [=]() { start_checkbox->setChecked(false); });

        if (publisher->availableActions().empty())
        {
          QFrame* empty = new QFrame(ui->framePublishers);
          empty->setFixedSize({ 22, 22 });
          ui->layoutPublishers->addWidget(empty, row, 2);
        }
        else
        {
          auto options_button = new QPushButton(ui->framePublishers);
          options_button->setFlat(true);
          options_button->setFixedSize({ 24, 24 });
          ui->layoutPublishers->addWidget(options_button, row, 2);

          options_button->setIcon(LoadSvg(":/resources/svg/settings_cog.svg", "light"));
          options_button->setIconSize({ 16, 16 });

          auto optionsMenu = [=]() {
            PopupMenu* menu = new PopupMenu(options_button, this);
            for (auto action : publisher->availableActions())
            {
              menu->addAction(action);
            }
            menu->exec();
          };

          connect(options_button, &QPushButton::clicked, options_button, optionsMenu);

          connect(this, &MainWindow::stylesheetChanged, options_button,
                  [=](QString style) {
                    options_button->setIcon(
                        LoadSvg(":/resources/svg/settings_cog.svg", style));
                  });
        }
      }
      else if (message_parser)
      {
        QStringList encodings = QString(message_parser->encoding()).split(";");
        auto parser_ptr = std::shared_ptr<ParserFactoryPlugin>(message_parser);
        for (const QString& encoding : encodings)
        {
          _parser_factories[encoding] = parser_ptr;
        }
      }
      else if (streamer)
      {
        if (_default_streamer == fileinfo.baseName())
        {
          _default_streamer = plugin_name;
        }
        _data_streamer.insert(std::make_pair(plugin_name, streamer));

        connect(streamer, &DataStreamer::closed, this,
                [this]() { this->stopStreamingPlugin(); });

        connect(streamer, &DataStreamer::clearBuffers, this,
                &MainWindow::on_actionClearBuffer_triggered);

        connect(streamer, &DataStreamer::dataReceived, _animated_streaming_movie,
                [this]() {
                  _animated_streaming_movie->start();
                  _animated_streaming_timer->start(500);
                });

        connect(streamer, &DataStreamer::removeGroup, this,
                &MainWindow::on_deleteSerieFromGroup);

        connect(streamer, &DataStreamer::dataReceived, this, [this]() {
          if (isStreamingActive() && !_replot_timer->isActive())
          {
            _replot_timer->setSingleShot(true);
            _replot_timer->start(40);
          }
        });

        connect(streamer, &DataStreamer::notificationsChanged, this,
                &MainWindow::on_streamingNotificationsChanged);
      }
      else if (toolbox)
      {
        toolbox->init(_mapped_plot_data, _transform_functions);

        _toolboxes.insert(std::make_pair(plugin_name, toolbox));

        auto action = ui->menuTools->addAction(toolbox->name());

        int new_index = ui->widgetStack->count();
        auto provided = toolbox->providedWidget();
        auto widget = provided.first;
        ui->widgetStack->addWidget(widget);

        connect(action, &QAction::triggered, toolbox, &ToolboxPlugin::onShowWidget);

        connect(action, &QAction::triggered, this,
                [=]() { ui->widgetStack->setCurrentIndex(new_index); });

        connect(toolbox, &ToolboxPlugin::closed, this,
                [=]() { ui->widgetStack->setCurrentIndex(0); });

        connect(toolbox, &ToolboxPlugin::importData, this,
                [this](PlotDataMapRef& new_data, bool remove_old) {
                  importPlotDataMap(new_data, remove_old);
                  updateDataAndReplot(true);
                });

        connect(toolbox, &ToolboxPlugin::plotCreated, this,
                [=](std::string name, bool is_custom) {
                  if (is_custom)
                  {
                    _curvelist_widget->addCustom(QString::fromStdString(name));
                  }
                  else
                  {
                    _curvelist_widget->addCurve(name);
                  }
                  _curvelist_widget->updateAppearance();
                  _curvelist_widget->clearSelections();
                });
      }
    }
    else
    {
      if (pluginLoader.errorString().contains("is not an ELF object") == false)
      {
        qDebug() << filename << ": " << pluginLoader.errorString();
      }
    }
  }

  for (auto& [name, streamer] : _data_streamer)
  {
    streamer->setParserFactories(&_parser_factories);
  }

  for (auto& [name, toolbox] : _toolboxes)
  {
    toolbox->setParserFactories(&_parser_factories);
  }

  for (auto& [name, loader] : _data_loader)
  {
    loader->setParserFactories(&_parser_factories);
  }

  if (!_data_streamer.empty())
  {
    QSignalBlocker block(ui->comboStreaming);
    ui->comboStreaming->setEnabled(true);
    ui->buttonStreamingStart->setEnabled(true);

    for (const auto& it : _data_streamer)
    {
      if (ui->comboStreaming->findText(it.first) == -1)
      {
        ui->comboStreaming->addItem(it.first);
      }
    }

    // remember the previous one
    QSettings settings;
    QString streaming_name =
        settings
            .value("MainWindow.previousStreamingPlugin", ui->comboStreaming->itemText(0))
            .toString();

    auto streamer_it = _data_streamer.find(streaming_name);
    if (streamer_it == _data_streamer.end())
    {
      streamer_it = _data_streamer.begin();
      streaming_name = streamer_it->first;
    }

    ui->comboStreaming->setCurrentText(streaming_name);

    bool contains_options = !streamer_it->second->availableActions().empty();
    ui->buttonStreamingOptions->setEnabled(contains_options);
  }
  qDebug() << "Number of plugins loaded: " << loaded_count << "\n";
  return loaded_out;
}

void MainWindow::buildDummyData()
{
  PlotDataMapRef datamap;
  BuildDummyData(datamap);
  importPlotDataMap(datamap, true);
}

void MainWindow::on_splitterMoved(int, int)
{
  QList<int> sizes = ui->mainSplitter->sizes();
  int max_left_size = _curvelist_widget->maximumWidth();
  int totalWidth = sizes[0] + sizes[1];

  // this is needed only once to restore the old size
  static bool first = true;
  if (sizes[0] != 0 && first)
  {
    first = false;
    QSettings settings;
    int splitter_width = settings.value("MainWindow.splitterWidth", 200).toInt();
    auto sizes = ui->mainSplitter->sizes();
    int tot_splitter_width = sizes[0] + sizes[1];
    sizes[0] = splitter_width;
    sizes[1] = tot_splitter_width - splitter_width;
    ui->mainSplitter->setSizes(sizes);
    return;
  }

  if (sizes[0] > max_left_size)
  {
    sizes[0] = max_left_size;
    sizes[1] = totalWidth - max_left_size;
    ui->mainSplitter->setSizes(sizes);
  }
}

void MainWindow::resizeEvent(QResizeEvent*)
{
  on_splitterMoved(0, 0);
}

void MainWindow::onPlotAdded(PlotWidget* plot)
{
  connect(plot, &PlotWidget::undoableChange, this, &MainWindow::onUndoableChange);

  connect(plot, &PlotWidget::trackerMoved, this, &MainWindow::onTrackerMovedFromWidget);

  connect(this, &MainWindow::dataSourceRemoved, plot, &PlotWidget::onDataSourceRemoved);

  connect(plot, &PlotWidget::curveListChanged, this, [this]() {
    updateTimeOffset();
    updateTimeSlider();
  });

  connect(&_time_offset, SIGNAL(valueChanged(double)), plot,
          SLOT(on_changeTimeOffset(double)));

  connect(ui->buttonUseDateTime, &QPushButton::toggled, plot,
          &PlotWidget::on_changeDateTimeScale);

  connect(plot, &PlotWidget::curvesDropped, _curvelist_widget,
          &CurveListPanel::clearSelections);

  connect(plot, &PlotWidget::legendSizeChanged, this, [=](int point_size) {
    auto visitor = [=](PlotWidget* p) {
      if (plot != p)
        p->setLegendSize(point_size);
    };
    this->forEachWidget(visitor);
  });

  connect(plot, &PlotWidget::rectChanged, this, &MainWindow::onPlotZoomChanged);

  plot->on_changeTimeOffset(_time_offset.get());
  plot->on_changeDateTimeScale(ui->buttonUseDateTime->isChecked());
  plot->activateGrid(ui->buttonActivateGrid->isChecked());
  plot->enableTracker(!isStreamingActive());
  plot->setKeepRatioXY(ui->buttonRatio->isChecked());
  plot->configureTracker(_tracker_param);
}

void MainWindow::onPlotZoomChanged(PlotWidget* modified_plot, QRectF new_range)
{
  if (ui->buttonLink->isChecked())
  {
    auto visitor = [=](PlotWidget* plot) {
      if (plot != modified_plot && !plot->isEmpty() && !plot->isXYPlot() &&
          plot->isZoomLinkEnabled())
      {
        QRectF bound_act = plot->currentBoundingRect();
        bound_act.setLeft(new_range.left());
        bound_act.setRight(new_range.right());
        plot->setZoomRectangle(bound_act, false);
        plot->on_zoomOutVertical_triggered(false);
        plot->replot();
      }
    };
    this->forEachWidget(visitor);
  }

  onUndoableChange();
}

void MainWindow::onPlotTabAdded(PlotDocker* docker)
{
  connect(docker, &PlotDocker::plotWidgetAdded, this, &MainWindow::onPlotAdded);

  connect(this, &MainWindow::stylesheetChanged, docker,
          &PlotDocker::on_stylesheetChanged);

  // TODO  connect(matrix, &PlotMatrix::undoableChange, this,
  // &MainWindow::onUndoableChange);
}

QDomDocument MainWindow::xmlSaveState() const
{
  QDomDocument doc;
  QDomProcessingInstruction instr = doc.createProcessingInstruction("xml", "version='1.0'"
                                                                           " encoding='"
                                                                           "UTF-8'");

  doc.appendChild(instr);

  QDomElement root = doc.createElement("root");

  for (auto& it : TabbedPlotWidget::instances())
  {
    QDomElement tabbed_area = it.second->xmlSaveState(doc);
    root.appendChild(tabbed_area);
  }

  doc.appendChild(root);

  QDomElement relative_time = doc.createElement("use_relative_time_offset");
  relative_time.setAttribute("enabled", ui->buttonRemoveTimeOffset->isChecked());
  root.appendChild(relative_time);

  return doc;
}

void MainWindow::checkAllCurvesFromLayout(const QDomElement& root)
{
  std::set<std::string> curves;

  for (QDomElement tw = root.firstChildElement("tabbed_widget"); !tw.isNull();
       tw = tw.nextSiblingElement("tabbed_widget"))
  {
    for (QDomElement pm = tw.firstChildElement("plotmatrix"); !pm.isNull();
         pm = pm.nextSiblingElement("plotmatrix"))
    {
      for (QDomElement pl = pm.firstChildElement("plot"); !pl.isNull();
           pl = pl.nextSiblingElement("plot"))
      {
        QDomElement tran_elem = pl.firstChildElement("transform");
        std::string trans = tran_elem.attribute("value").toStdString();
        bool is_XY_plot = (trans == "XYPlot");

        for (QDomElement cv = pl.firstChildElement("curve"); !cv.isNull();
             cv = cv.nextSiblingElement("curve"))
        {
          if (is_XY_plot)
          {
            curves.insert(cv.attribute("curve_x").toStdString());
            curves.insert(cv.attribute("curve_y").toStdString());
          }
          else
          {
            curves.insert(cv.attribute("name").toStdString());
          }
        }
      }
    }
  }

  std::vector<std::string> missing_curves;

  for (auto& curve_name : curves)
  {
    if (_mapped_plot_data.numeric.count(curve_name) == 0)
    {
      missing_curves.push_back(curve_name);
    }
    if (_mapped_plot_data.strings.count(curve_name) == 0)
    {
      missing_curves.push_back(curve_name);
    }
  }
  if (missing_curves.size() > 0)
  {
    QMessageBox msgBox(this);
    msgBox.setWindowTitle("Warning");
    msgBox.setText(tr("One or more timeseries in the layout haven't been loaded yet\n"
                      "What do you want to do?"));

    QPushButton* buttonRemove =
        msgBox.addButton(tr("Remove curves from plots"), QMessageBox::RejectRole);
    QPushButton* buttonPlaceholder =
        msgBox.addButton(tr("Create empty placeholders"), QMessageBox::YesRole);
    msgBox.setDefaultButton(buttonPlaceholder);
    msgBox.exec();
    if (msgBox.clickedButton() == buttonPlaceholder)
    {
      for (auto& name : missing_curves)
      {
        auto plot_it = _mapped_plot_data.addNumeric(name);
        _curvelist_widget->addCurve(name);
      }
      _curvelist_widget->refreshColumns();
    }
  }
}

bool MainWindow::xmlLoadState(QDomDocument state_document)
{
  QDomElement root = state_document.namedItem("root").toElement();
  if (root.isNull())
  {
    qWarning() << "No <root> element found at the top-level of the XML file!";
    return false;
  }

  size_t num_floating = 0;
  std::map<QString, QDomElement> tabbed_widgets_with_name;

  for (QDomElement tw = root.firstChildElement("tabbed_widget"); tw.isNull() == false;
       tw = tw.nextSiblingElement("tabbed_widget"))
  {
    if (tw.attribute("parent") != ("main_window"))
    {
      num_floating++;
    }
    tabbed_widgets_with_name[tw.attribute("name")] = tw;
  }

  // add if missing
  for (const auto& it : tabbed_widgets_with_name)
  {
    if (TabbedPlotWidget::instance(it.first) == nullptr)
    {
      // TODO createTabbedDialog(it.first, nullptr);
    }
  }

  // remove those which don't share list of names
  for (const auto& it : TabbedPlotWidget::instances())
  {
    if (tabbed_widgets_with_name.count(it.first) == 0)
    {
      it.second->deleteLater();
    }
  }

  //-----------------------------------------------------
  checkAllCurvesFromLayout(root);
  //-----------------------------------------------------

  for (QDomElement tw = root.firstChildElement("tabbed_widget"); tw.isNull() == false;
       tw = tw.nextSiblingElement("tabbed_widget"))
  {
    TabbedPlotWidget* tabwidget = TabbedPlotWidget::instance(tw.attribute("name"));
    tabwidget->xmlLoadState(tw);
  }

  QDomElement relative_time = root.firstChildElement("use_relative_time_offset");
  if (!relative_time.isNull())
  {
    bool remove_offset = (relative_time.attribute("enabled") == QString("1"));
    ui->buttonRemoveTimeOffset->setChecked(remove_offset);
  }
  return true;
}

void MainWindow::onDeleteMultipleCurves(const std::vector<std::string>& curve_names)
{
  std::set<std::string> to_be_deleted;
  for (auto& name : curve_names)
  {
    to_be_deleted.insert(name);
  }
  // add to the list of curves to delete the derived transforms
  size_t prev_size = 0;
  while (prev_size < to_be_deleted.size())
  {
    prev_size = to_be_deleted.size();
    for (auto& [trans_name, transform] : _transform_functions)
    {
      for (const auto& source : transform->dataSources())
      {
        if (to_be_deleted.count(source->plotName()) > 0)
        {
          to_be_deleted.insert(trans_name);
        }
      }
    }
  }

  for (const auto& curve_name : to_be_deleted)
  {
    emit dataSourceRemoved(curve_name);
    _curvelist_widget->removeCurve(curve_name);
    _mapped_plot_data.erase(curve_name);
    _transform_functions.erase(curve_name);
  }
  updateTimeOffset();
  forEachWidget([](PlotWidget* plot) { plot->replot(); });
}

void MainWindow::updateRecentDataMenu(QStringList new_filenames)
{
  QMenu* menu = _recent_data_files;

  QAction* separator = nullptr;
  QStringList prev_filenames;
  for (QAction* action : menu->actions())
  {
    if (action->isSeparator())
    {
      separator = action;
      break;
    }
    if (new_filenames.contains(action->text()) == false)
    {
      prev_filenames.push_back(action->text());
    }
    menu->removeAction(action);
  }

  new_filenames.append(prev_filenames);
  while (new_filenames.size() > 10)
  {
    new_filenames.removeLast();
  }

  for (const auto& filename : new_filenames)
  {
    QAction* action = new QAction(filename, nullptr);
    connect(action, &QAction::triggered, this,
            [this, filename] { loadDataFromFiles({ filename }); });
    menu->insertAction(separator, action);
  }

  QSettings settings;
  settings.setValue("MainWindow.recentlyLoadedDatafile", new_filenames);
  menu->setEnabled(new_filenames.size() > 0);
}

void MainWindow::updateRecentLayoutMenu(QStringList new_filenames)
{
  QMenu* menu = _recent_layout_files;

  QAction* separator = nullptr;
  QStringList prev_filenames;
  for (QAction* action : menu->actions())
  {
    if (action->isSeparator())
    {
      separator = action;
      break;
    }
    if (new_filenames.contains(action->text()) == false)
    {
      prev_filenames.push_back(action->text());
    }
    menu->removeAction(action);
  }

  new_filenames.append(prev_filenames);
  while (new_filenames.size() > 10)
  {
    new_filenames.removeLast();
  }

  for (const auto& filename : new_filenames)
  {
    QAction* action = new QAction(filename, nullptr);
    connect(action, &QAction::triggered, this, [this, filename] {
      if (this->loadLayoutFromFile(filename))
      {
        updateRecentLayoutMenu({ filename });
      }
    });
    menu->insertAction(separator, action);
  }

  QSettings settings;
  settings.setValue("MainWindow.recentlyLoadedLayout", new_filenames);
  menu->setEnabled(new_filenames.size() > 0);
}

void MainWindow::deleteAllData()
{
  forEachWidget([](PlotWidget* plot) { plot->removeAllCurves(); });

  _mapped_plot_data.clear();
  _transform_functions.clear();
  _curvelist_widget->clear();
  _loaded_datafiles_history.clear();
  _undo_states.clear();
  _redo_states.clear();

  bool stopped = false;

  for (int idx = 0; idx < ui->layoutPublishers->count(); idx++)
  {
    QLayoutItem* item = ui->layoutPublishers->itemAt(idx);
    if (dynamic_cast<QWidgetItem*>(item))
    {
      if (auto checkbox = dynamic_cast<QCheckBox*>(item->widget()))
      {
        if (checkbox->isChecked())
        {
          checkbox->setChecked(false);
          stopped = true;
        }
      }
    }
  }

  if (stopped)
  {
    QMessageBox::warning(this, "State publishers stopped",
                         "All the state publishers have been stopped because old data "
                         "has been deleted.");
  }
}

void MainWindow::importPlotDataMap(PlotDataMapRef& new_data, bool remove_old)
{
  if (remove_old)
  {
    auto ClearOldSeries = [](auto& prev_plot_data, auto& new_plot_data) {
      for (auto& it : prev_plot_data)
      {
        // timeseries in both
        if (new_plot_data.count(it.first) != 0)
        {
          it.second.clear();
        }
      }
    };

    ClearOldSeries(_mapped_plot_data.scatter_xy, new_data.scatter_xy);
    ClearOldSeries(_mapped_plot_data.numeric, new_data.numeric);
    ClearOldSeries(_mapped_plot_data.strings, new_data.strings);
  }

  auto [added_curves, curve_updated, data_pushed] =
      MoveData(new_data, _mapped_plot_data, remove_old);

  for (const auto& added_curve : added_curves)
  {
    _curvelist_widget->addCurve(added_curve);
  }

  if (curve_updated)
  {
    _curvelist_widget->refreshColumns();
  }
}

bool MainWindow::isStreamingActive() const
{
  return !ui->buttonStreamingPause->isChecked() && _active_streamer_plugin;
}

bool MainWindow::loadDataFromFiles(QStringList filenames)
{
  filenames.sort();
  std::map<QString, QString> filename_prefix;

  if (filenames.size() > 1 || ui->checkBoxAddPrefixAndMerge->isChecked())
  {
    DialogMultifilePrefix dialog(filenames, this);
    int ret = dialog.exec();
    if (ret != QDialog::Accepted)
    {
      return false;
    }
    filename_prefix = dialog.getPrefixes();
  }

  std::unordered_set<std::string> previous_names = _mapped_plot_data.getAllNames();

  QStringList loaded_filenames;
  _loaded_datafiles_previous.clear();

  for (int i = 0; i < filenames.size(); i++)
  {
    FileLoadInfo info;
    info.filename = filenames[i];
    if (filename_prefix.count(info.filename) > 0)
    {
      info.prefix = filename_prefix[info.filename];
    }
//    auto added_names = loadDataFromFile(info);
    auto added_names = loadData_opengl(filenames[i]);
    if (!added_names.empty())
    {
      loaded_filenames.push_back(filenames[i]);
    }
    for (const auto& name : added_names)
    {
      previous_names.erase(name);
    }
  }

  bool data_replaced_entirely = false;

  if (previous_names.empty())
  {
    data_replaced_entirely = true;
  }
  else if (!ui->checkBoxAddPrefixAndMerge->isChecked())
  {
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(
        this, tr("Warning"), tr("Do you want to remove the previously loaded data?\n"),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::NoButton);

    if (reply == QMessageBox::Yes)
    {
      std::vector<std::string> to_delete;
      for (const auto& name : previous_names)
      {
        to_delete.push_back(name);
      }
      onDeleteMultipleCurves(to_delete);
      data_replaced_entirely = true;
    }
  }

  // special case when only the last file should be remembered
  if (loaded_filenames.size() == 1 && data_replaced_entirely &&
      _loaded_datafiles_history.size() > 1)
  {
    std::swap(_loaded_datafiles_history.back(), _loaded_datafiles_history.front());
    _loaded_datafiles_history.resize(1);
  }

  ui->buttonReloadData->setEnabled(!loaded_filenames.empty());

  if (loaded_filenames.size() > 0)
  {
    updateRecentDataMenu(loaded_filenames);
    linkedZoomOut();
    return true;
  }
  return false;
}

std::unordered_set<std::string> MainWindow::loadDataFromFile(const FileLoadInfo& info)
{
  ui->buttonPlay->setChecked(false);

  const QString extension = QFileInfo(info.filename).suffix().toLower();

  typedef std::map<QString, DataLoaderPtr>::iterator MapIterator;

  std::vector<MapIterator> compatible_loaders;

  for (auto it = _data_loader.begin(); it != _data_loader.end(); ++it)
  {
    DataLoaderPtr data_loader = it->second;
    std::vector<const char*> extensions = data_loader->compatibleFileExtensions();

    for (auto& ext : extensions)
    {
      if (extension == QString(ext).toLower())
      {
        compatible_loaders.push_back(it);
        break;
      }
    }
  }

  DataLoaderPtr dataloader;
  std::unordered_set<std::string> added_names;

  if (compatible_loaders.size() == 1)
  {
    dataloader = compatible_loaders.front()->second;
  }
  else
  {
    static QString last_plugin_name_used;

    QStringList names;
    for (auto& cl : compatible_loaders)
    {
      const auto& name = cl->first;

      if (name == last_plugin_name_used)
      {
        names.push_front(name);
      }
      else
      {
        names.push_back(name);
      }
    }

    bool ok;
    QString plugin_name =
        QInputDialog::getItem(this, tr("QInputDialog::getItem()"),
                              tr("Select the loader to use:"), names, 0, false, &ok);
    if (ok && !plugin_name.isEmpty() &&
        (_enabled_plugins.size() == 0 || _enabled_plugins.contains(plugin_name)))
    {
      dataloader = _data_loader[plugin_name];
      last_plugin_name_used = plugin_name;
    }
  }

  if (dataloader)
  {
    QFile file(info.filename);

    if (!file.open(QFile::ReadOnly | QFile::Text))
    {
      QMessageBox::warning(
          this, tr("Datafile"),
          tr("Cannot read file %1:\n%2.").arg(info.filename).arg(file.errorString()));
      return {};
    }
    file.close();

    try
    {
      PlotDataMapRef mapped_data;
      FileLoadInfo new_info = info;

      if (info.plugin_config.hasChildNodes())
      {
        dataloader->xmlLoadState(info.plugin_config.firstChildElement());
      }

      if (dataloader->readDataFromFile(&new_info, mapped_data))
      {
        AddPrefixToPlotData(info.prefix.toStdString(), mapped_data.numeric);
        AddPrefixToPlotData(info.prefix.toStdString(), mapped_data.strings);

        added_names = mapped_data.getAllNames();
        importPlotDataMap(mapped_data, true);

        QDomElement plugin_elem = dataloader->xmlSaveState(new_info.plugin_config);
        new_info.plugin_config.appendChild(plugin_elem);
        _loaded_datafiles_previous.push_back(new_info);

        bool duplicate = false;

        // substitute an old item of _loaded_datafiles or push_back another item.
        for (auto& prev_loaded : _loaded_datafiles_history)
        {
          if (prev_loaded.filename == new_info.filename &&
              prev_loaded.prefix == new_info.prefix)
          {
            prev_loaded = new_info;
            duplicate = true;
            break;
          }
        }

        if (!duplicate)
        {
          _loaded_datafiles_history.push_back(new_info);
        }
      }
    }
    catch (std::exception& ex)
    {
      QMessageBox::warning(this, tr("Exception from the plugin"),
                           tr("The plugin [%1] thrown the following exception: \n\n %3\n")
                               .arg(dataloader->name())
                               .arg(ex.what()));
      return {};
    }
  }
  else
  {
    QMessageBox::warning(this, tr("Error"),
                         tr("Cannot read files with extension %1.\n No plugin can handle "
                            "that!\n")
                             .arg(info.filename));
  }
  _curvelist_widget->updateFilter();

  // clean the custom plot. Function updateDataAndReplot will update them
  for (auto& custom_it : _transform_functions)
  {
    auto it = _mapped_plot_data.numeric.find(custom_it.first);
    if (it != _mapped_plot_data.numeric.end())
    {
      it->second.clear();
    }
    custom_it.second->reset();
  }
  forEachWidget([](PlotWidget* plot) { plot->updateCurves(true); });

  updateDataAndReplot(true);
  ui->timeSlider->setRealValue(ui->timeSlider->getMinimum());

  return added_names;
}

std::unordered_set<std::string> MainWindow::loadData_opengl(const QString &filePath)
{
    std::map<std::string, std::vector<std::pair<double, double>>> plot_data;
    plot_data = glWidget->SetFilePath(filePath);



  ui->buttonPlay->setChecked(false);

  const QString extension = "csv";

  typedef std::map<QString, DataLoaderPtr>::iterator MapIterator;

  std::vector<MapIterator> compatible_loaders;

  for (auto it = _data_loader.begin(); it != _data_loader.end(); ++it)
  {
    DataLoaderPtr data_loader = it->second;
    std::vector<const char*> extensions = data_loader->compatibleFileExtensions();

    for (auto& ext : extensions)
    {
      if (extension == QString(ext).toLower())
      {
        compatible_loaders.push_back(it);
        break;
      }
    }
  }

  DataLoaderPtr dataloader;
  std::unordered_set<std::string> added_names;

  if (compatible_loaders.size() == 1)
  {
    dataloader = compatible_loaders.front()->second;
  }
  else
  {
    static QString last_plugin_name_used;

    QStringList names;
    for (auto& cl : compatible_loaders)
    {
      const auto& name = cl->first;

      if (name == last_plugin_name_used)
      {
        names.push_front(name);
      }
      else
      {
        names.push_back(name);
      }
    }

    bool ok;
    QString plugin_name =
        QInputDialog::getItem(this, tr("QInputDialog::getItem()"),
                              tr("Select the loader to use:"), names, 0, false, &ok);
    if (ok && !plugin_name.isEmpty() &&
        (_enabled_plugins.size() == 0 || _enabled_plugins.contains(plugin_name)))
    {
      dataloader = _data_loader[plugin_name];
      last_plugin_name_used = plugin_name;
    }
  }

  if (dataloader)
  {

    try
    {
      PlotDataMapRef mapped_data;

      dataloader->loadData(mapped_data, plot_data);

      AddPrefixToPlotData("", mapped_data.numeric);

      added_names = mapped_data.getAllNames();
      importPlotDataMap(mapped_data, true);

    }
    catch (std::exception& ex)
    {
      QMessageBox::warning(this, tr("Exception from the plugin"),
                           tr("The plugin [%1] thrown the following exception: \n\n %3\n")
                               .arg(dataloader->name())
                               .arg(ex.what()));
      return {};
    }
  }

  _curvelist_widget->updateFilter();

  // clean the custom plot. Function updateDataAndReplot will update them
  for (auto& custom_it : _transform_functions)
  {
    auto it = _mapped_plot_data.numeric.find(custom_it.first);
    if (it != _mapped_plot_data.numeric.end())
    {
      it->second.clear();
    }
    custom_it.second->reset();
  }
  forEachWidget([](PlotWidget* plot) { plot->updateCurves(true); });

  updateDataAndReplot(true);
  ui->timeSlider->setRealValue(ui->timeSlider->getMinimum());

  return added_names;
}

void MainWindow::on_buttonStreamingNotifications_clicked()
{
  if (_data_streamer.empty())
  {
    return;
  }
  auto streamer = _data_streamer.at(ui->comboStreaming->currentText());
  QAction* notification_button_action = streamer->notificationAction().first;
  if (notification_button_action != nullptr)
  {
    notification_button_action->trigger();
  }
}

void MainWindow::on_buttonStreamingPause_toggled(bool paused)
{
  if (!_active_streamer_plugin)
  {
    paused = true;
  }

  ui->buttonRemoveTimeOffset->setEnabled(paused);
  ui->widgetPlay->setEnabled(paused);

  if (!paused && ui->buttonPlay->isChecked())
  {
    ui->buttonPlay->setChecked(false);
  }

  forEachWidget([&](PlotWidget* plot) {
    plot->enableTracker(paused);
    plot->setZoomEnabled(paused);
  });

  if (!paused)
  {
    updateTimeOffset();
  }
  else
  {
    onUndoableChange();
  }
}

void MainWindow::on_streamingToggled()
{
  if (_active_streamer_plugin)
  {
    bool prev_state = ui->buttonStreamingPause->isChecked();
    ui->buttonStreamingPause->setChecked(!prev_state);
  }
}

void MainWindow::stopStreamingPlugin()
{
  ui->comboStreaming->setEnabled(true);
  ui->buttonStreamingStart->setText("Start");
  ui->buttonStreamingPause->setEnabled(false);
  ui->labelStreamingAnimation->setHidden(true);
  enableStreamingNotificationsButton(false);

  // force the cleanups typically done in on_buttonStreamingPause_toggled
  if (ui->buttonStreamingPause->isChecked())
  {
    // Will call on_buttonStreamingPause_toggled
    ui->buttonStreamingPause->setChecked(false);
  }
  else
  {
    // call it manually
    on_buttonStreamingPause_toggled(true);
  }

  if (_active_streamer_plugin)
  {
    _active_streamer_plugin->shutdown();
    _active_streamer_plugin = nullptr;
  }

  if (!_mapped_plot_data.numeric.empty())
  {
    ui->actionDeleteAllData->setToolTip("");
  }

  // reset max range.
  _mapped_plot_data.setMaximumRangeX(std::numeric_limits<double>::max());
}

void MainWindow::startStreamingPlugin(QString streamer_name)
{
  if (_active_streamer_plugin)
  {
    _active_streamer_plugin->shutdown();
    _active_streamer_plugin = nullptr;
  }

  if (_data_streamer.empty())
  {
    qDebug() << "Error, no streamer loaded";
    return;
  }

  auto it = _data_streamer.find(streamer_name);
  if (it != _data_streamer.end())
  {
    _active_streamer_plugin = it->second;
  }
  else
  {
    qDebug() << "Error. The streamer " << streamer_name << " can't be loaded";
    _active_streamer_plugin = nullptr;
    return;
  }

  bool started = false;
  try
  {
    // TODO data sources (argument to _active_streamer_plugin->start()
    started = _active_streamer_plugin && _active_streamer_plugin->start(nullptr);
  }
  catch (std::runtime_error& err)
  {
    QMessageBox::warning(
        this, tr("Exception from the plugin"),
        tr("The plugin thrown the following exception: \n\n %1\n").arg(err.what()));
    _active_streamer_plugin = nullptr;
    return;
  }

  // The attemp to start the plugin may have succeded or failed
  if (started)
  {
    {
      std::lock_guard<std::mutex> lock(_active_streamer_plugin->mutex());
      importPlotDataMap(_active_streamer_plugin->dataMap(), false);
    }

    ui->actionClearBuffer->setEnabled(true);
    ui->actionDeleteAllData->setToolTip("Stop streaming to be able to delete the data");

    ui->buttonStreamingStart->setText("Stop");
    ui->buttonStreamingPause->setEnabled(true);
    ui->buttonStreamingPause->setChecked(false);
    ui->comboStreaming->setEnabled(false);
    ui->labelStreamingAnimation->setHidden(false);

    // force start
    on_buttonStreamingPause_toggled(false);
    // this will force the update the max buffer size values
    on_streamingSpinBox_valueChanged(ui->streamingSpinBox->value());
  }
  else
  {
    QSignalBlocker block(ui->buttonStreamingStart);
    ui->buttonStreamingStart->setChecked(false);
    qDebug() << "Failed to launch the streamer";
    _active_streamer_plugin = nullptr;
  }
}

void MainWindow::enableStreamingNotificationsButton(bool enabled)
{
  ui->buttonStreamingNotifications->setEnabled(enabled);

  QSettings settings;
  QString theme = settings.value("Preferences::theme", "light").toString();

  if (enabled)
  {
    ui->buttonStreamingNotifications->setIcon(
        LoadSvg(":/resources/svg/alarm-bell-active.svg", theme));
  }
  else
  {
    ui->buttonStreamingNotifications->setIcon(
        LoadSvg(":/resources/svg/alarm-bell.svg", theme));
  }
}

void MainWindow::setStatusBarMessage(QString message)
{
  ui->statusLabel->setText(message);
  ui->widgetStatusBar->setHidden(message.isEmpty());
  QTimer::singleShot(7000, this, [this]() { ui->widgetStatusBar->setHidden(true); });
}

void MainWindow::loadStyleSheet(QString file_path)
{
  QFile styleFile(file_path);
  styleFile.open(QFile::ReadOnly);
  try
  {
    QString theme = SetApplicationStyleSheet(styleFile.readAll());

    forEachWidget([&](PlotWidget* plot) { plot->replot(); });

    _curvelist_widget->updateAppearance();
    emit stylesheetChanged(theme);
  }
  catch (std::runtime_error& err)
  {
    QMessageBox::warning(this, tr("Error loading StyleSheet"), tr(err.what()));
  }
}

void MainWindow::updateDerivedSeries()
{
  for (auto& [id, series] : _transform_functions)
  {
  }
}

void MainWindow::updateReactivePlots()
{
  std::unordered_set<std::string> updated_curves;

  bool curve_added = false;
  for (auto& it : _transform_functions)
  {
    if (auto reactive_function =
            std::dynamic_pointer_cast<PJ::ReactiveLuaFunction>(it.second))
    {
      reactive_function->setTimeTracker(_tracker_time);
      reactive_function->calculate();

      for (auto& name : reactive_function->createdCurves())
      {
        curve_added |= _curvelist_widget->addCurve(name);
        updated_curves.insert(name);
      }
    }
  }
  if (curve_added)
  {
    _curvelist_widget->refreshColumns();
  }

  forEachWidget([&](PlotWidget* plot) {
    for (auto& curve : plot->curveList())
    {
      if (updated_curves.count(curve.src_name) != 0)
      {
        plot->replot();
      }
    }
  });
}

void MainWindow::dragEnterEvent(QDragEnterEvent* event)
{
  if (event->mimeData()->hasUrls())
  {
    event->acceptProposedAction();
  }
}

void MainWindow::dropEvent(QDropEvent* event)
{
  QStringList file_names;
  const auto urls = event->mimeData()->urls();

  for (const auto& url : urls)
  {
    file_names << QDir::toNativeSeparators(url.toLocalFile());
  }

  loadDataFromFiles(file_names);
}

void MainWindow::initOpenGLWidget()
{
    glWidget = new GLWidget;
    glWidget->setMinimumWidth(300);
    ui->mainSplitter->addWidget(glWidget);
}

void MainWindow::on_stylesheetChanged(QString theme)
{
  ui->buttonLoadDatafile->setIcon(LoadSvg(":/resources/svg/import.svg", theme));
  ui->buttonStreamingPause->setIcon(LoadSvg(":/resources/svg/pause.svg", theme));
  if (ui->buttonStreamingNotifications->isEnabled())
  {
    ui->buttonStreamingNotifications->setIcon(
        LoadSvg(":/resources/svg/alarm-bell-active.svg", theme));
  }
  else
  {
    ui->buttonStreamingNotifications->setIcon(
        LoadSvg(":/resources/svg/alarm-bell.svg", theme));
  }
  ui->buttonRecentData->setIcon(LoadSvg(":/resources/svg/right-arrow.svg", theme));
  ui->buttonRecentLayout->setIcon(LoadSvg(":/resources/svg/right-arrow.svg", theme));

  ui->buttonZoomOut->setIcon(LoadSvg(":/resources/svg/zoom_max.svg", theme));
  ui->playbackLoop->setIcon(LoadSvg(":/resources/svg/loop.svg", theme));
  ui->buttonPlay->setIcon(LoadSvg(":/resources/svg/play_arrow.svg", theme));
  ui->buttonUseDateTime->setIcon(LoadSvg(":/resources/svg/datetime.svg", theme));
  ui->buttonActivateGrid->setIcon(LoadSvg(":/resources/svg/grid.svg", theme));
  ui->buttonRatio->setIcon(LoadSvg(":/resources/svg/ratio.svg", theme));

  ui->buttonLoadLayout->setIcon(LoadSvg(":/resources/svg/import.svg", theme));
  ui->buttonSaveLayout->setIcon(LoadSvg(":/resources/svg/export.svg", theme));

  ui->buttonLink->setIcon(LoadSvg(":/resources/svg/link.svg", theme));
  ui->buttonRemoveTimeOffset->setIcon(LoadSvg(":/resources/svg/t0.svg", theme));
  ui->buttonLegend->setIcon(LoadSvg(":/resources/svg/legend.svg", theme));

  ui->buttonStreamingOptions->setIcon(LoadSvg(":/resources/svg/settings_cog.svg", theme));
}

void MainWindow::loadPluginState(const QDomElement& root)
{
  QDomElement plugins = root.firstChildElement("Plugins");

  for (QDomElement plugin_elem = plugins.firstChildElement();
       plugin_elem.isNull() == false; plugin_elem = plugin_elem.nextSiblingElement())
  {
    const QString plugin_name = plugin_elem.attribute("ID");

    if (plugin_elem.nodeName() != "plugin" || plugin_name.isEmpty())
    {
      QMessageBox::warning(this, tr("Error loading Plugin State from Layout"),
                           tr("The method xmlSaveState() must return a node like this "
                              "<plugin ID=\"PluginName\" "));
    }

    if (_data_loader.find(plugin_name) != _data_loader.end())
    {
      _data_loader[plugin_name]->xmlLoadState(plugin_elem);
    }
    if (_data_streamer.find(plugin_name) != _data_streamer.end())
    {
      _data_streamer[plugin_name]->xmlLoadState(plugin_elem);
    }
    if (_toolboxes.find(plugin_name) != _toolboxes.end())
    {
      _toolboxes[plugin_name]->xmlLoadState(plugin_elem);
    }
    if (_state_publisher.find(plugin_name) != _state_publisher.end())
    {
      StatePublisherPtr publisher = _state_publisher[plugin_name];
      publisher->xmlLoadState(plugin_elem);

      if (_autostart_publishers && plugin_elem.attribute("status") == "active")
      {
        publisher->setEnabled(true);
      }
    }
  }
}

QDomElement MainWindow::savePluginState(QDomDocument& doc)
{
  QDomElement list_plugins = doc.createElement("Plugins");

  auto AddPlugins = [&](auto& plugins) {
    for (auto& [name, plugin] : plugins)
    {
      QDomElement elem = plugin->xmlSaveState(doc);
      list_plugins.appendChild(elem);
    }
  };

  AddPlugins(_data_loader);
  AddPlugins(_data_streamer);
  AddPlugins(_toolboxes);
  AddPlugins(_state_publisher);

  for (auto& it : _state_publisher)
  {
    const auto& state_publisher = it.second;
    QDomElement plugin_elem = state_publisher->xmlSaveState(doc);
    plugin_elem.setAttribute("status", state_publisher->enabled() ? "active" : "idle");
  }

  return list_plugins;
}

std::tuple<double, double, int> MainWindow::calculateVisibleRangeX()
{
  // find min max time
  double min_time = std::numeric_limits<double>::max();
  double max_time = std::numeric_limits<double>::lowest();
  int max_steps = 0;

  forEachWidget([&](const PlotWidget* widget) {
    for (auto& it : widget->curveList())
    {
      const auto& curve_name = it.src_name;

      auto plot_it = _mapped_plot_data.numeric.find(curve_name);
      if (plot_it == _mapped_plot_data.numeric.end())
      {
        continue;  // FIXME?
      }
      const auto& data = plot_it->second;
      if (data.size() >= 1)
      {
        const double t0 = data.front().x;
        const double t1 = data.back().x;
        min_time = std::min(min_time, t0);
        max_time = std::max(max_time, t1);
        max_steps = std::max(max_steps, (int)data.size());
      }
    }
  });

  // needed if all the plots are empty
  if (max_steps == 0 || max_time < min_time)
  {
    for (const auto& it : _mapped_plot_data.numeric)
    {
      const PlotData& data = it.second;
      if (data.size() >= 1)
      {
        const double t0 = data.front().x;
        const double t1 = data.back().x;
        min_time = std::min(min_time, t0);
        max_time = std::max(max_time, t1);
        max_steps = std::max(max_steps, (int)data.size());
      }
    }
  }

  // last opportunity. Everything else failed
  if (max_steps == 0 || max_time < min_time)
  {
    min_time = 0.0;
    max_time = 1.0;
    max_steps = 1;
  }
  return std::tuple<double, double, int>(min_time, max_time, max_steps);
}

bool MainWindow::loadLayoutFromFile(QString filename)
{
  QSettings settings;

  QFile file(filename);
  if (!file.open(QFile::ReadOnly | QFile::Text))
  {
    QMessageBox::warning(
        this, tr("Layout"),
        tr("Cannot read file %1:\n%2.").arg(filename).arg(file.errorString()));
    return false;
  }

  QString errorStr;
  int errorLine, errorColumn;

  QDomDocument domDocument;

  if (!domDocument.setContent(&file, true, &errorStr, &errorLine, &errorColumn))
  {
    QMessageBox::information(
        window(), tr("XML Layout"),
        tr("Parse error at line %1:\n%2").arg(errorLine).arg(errorStr));
    return false;
  }

  //-------------------------------------------------
  // refresh plugins
  QDomElement root = domDocument.namedItem("root").toElement();

  loadPluginState(root);
  //-------------------------------------------------
  QDomElement previously_loaded_datafile = root.firstChildElement("previouslyLoaded_"
                                                                  "Datafiles");

  QDomElement datafile_elem = previously_loaded_datafile.firstChildElement("fileInfo");
  while (!datafile_elem.isNull())
  {
    QString datafile_path = datafile_elem.attribute("filename");
    if (QDir(datafile_path).isRelative())
    {
      QDir layout_directory = QFileInfo(filename).absoluteDir();
      QString new_path = layout_directory.filePath(datafile_path);
      datafile_path = QFileInfo(new_path).absoluteFilePath();
    }

    FileLoadInfo info;
    info.filename = datafile_path;
    info.prefix = datafile_elem.attribute("prefix");

    auto plugin_elem = datafile_elem.firstChildElement("plugin");
    info.plugin_config.appendChild(info.plugin_config.importNode(plugin_elem, true));

    loadDataFromFile(info);
    datafile_elem = datafile_elem.nextSiblingElement("fileInfo");
  }

  QDomElement previous_streamer = root.firstChildElement("previouslyLoaded_Streamer");
  if (!previous_streamer.isNull())
  {
    QString streamer_name = previous_streamer.attribute("name");

    QMessageBox msgBox(this);
    msgBox.setWindowTitle("Start Streaming?");
    msgBox.setText(
        tr("Start the previously used streaming plugin?\n\n %1 \n\n").arg(streamer_name));
    QPushButton* yes = msgBox.addButton(tr("Yes"), QMessageBox::YesRole);
    QPushButton* no = msgBox.addButton(tr("No"), QMessageBox::RejectRole);
    msgBox.setDefaultButton(yes);
    msgBox.exec();

    if (msgBox.clickedButton() == yes)
    {
      if (_data_streamer.count(streamer_name) != 0)
      {
        auto allCurves = readAllCurvesFromXML(root);

        // create placeholders, if necessary
        for (auto curve_name : allCurves)
        {
          std::string curve_str = curve_name.toStdString();
          if (_mapped_plot_data.numeric.count(curve_str) == 0)
          {
            _mapped_plot_data.addNumeric(curve_str);
          }
        }

        startStreamingPlugin(streamer_name);
      }
      else
      {
        QMessageBox::warning(
            this, tr("Error Loading Streamer"),
            tr("The streamer named %1 can not be loaded.").arg(streamer_name));
      }
    }
  }
  //-------------------------------------------------
  // autostart_publishers
  QDomElement plugins = root.firstChildElement("Plugins");

  if (!plugins.isNull() && _autostart_publishers)
  {
    for (QDomElement plugin_elem = plugins.firstChildElement();
         plugin_elem.isNull() == false; plugin_elem = plugin_elem.nextSiblingElement())
    {
      const QString plugin_name = plugin_elem.nodeName();
      if (_state_publisher.find(plugin_name) != _state_publisher.end())
      {
        StatePublisherPtr publisher = _state_publisher[plugin_name];

        if (plugin_elem.attribute("status") == "active")
        {
          publisher->setEnabled(true);
        }
      }
    }
  }
  //-------------------------------------------------
  auto custom_equations = root.firstChildElement("customMathEquations");

  if (!custom_equations.isNull())
  {
    using SnippetPair = std::pair<SnippetData, QDomElement>;
    std::vector<SnippetPair> snippets;

    for (QDomElement custom_eq = custom_equations.firstChildElement("snippet");
         custom_eq.isNull() == false; custom_eq = custom_eq.nextSiblingElement("snippet"))
    {
      snippets.push_back({ GetSnippetFromXML(custom_eq), custom_eq });
    }
    // A custom plot may depend on other custom plots.
    // Reorder them to respect the mutual depencency.
    auto DependOn = [](const SnippetPair& a, const SnippetPair& b) {
      if (b.first.linked_source == a.first.alias_name)
      {
        return true;
      }
      for (const auto& source : b.first.additional_sources)
      {
        if (source == a.first.alias_name)
        {
          return true;
        }
      }
      return false;
    };
    std::sort(snippets.begin(), snippets.end(), DependOn);

    for (const auto& [snippet, custom_eq] : snippets)
    {
      try
      {
        CustomPlotPtr new_custom_plot = std::make_shared<LuaCustomFunction>(snippet);
        new_custom_plot->xmlLoadState(custom_eq);

        new_custom_plot->calculateAndAdd(_mapped_plot_data);
        const auto& alias_name = new_custom_plot->aliasName();
        _curvelist_widget->addCustom(alias_name);

        _transform_functions.insert({ alias_name.toStdString(), new_custom_plot });
      }
      catch (std::runtime_error& err)
      {
        QMessageBox::warning(this, tr("Exception"),
                             tr("Failed to load customMathEquation [%1] \n\n %2\n")
                                 .arg(snippet.alias_name)
                                 .arg(err.what()));
      }
    }
    _curvelist_widget->refreshColumns();
  }

  auto colormaps = root.firstChildElement("colorMaps");

  if (!colormaps.isNull())
  {
    for (auto colormap = colormaps.firstChildElement("colorMap");
         colormap.isNull() == false; colormap = colormap.nextSiblingElement("colorMap"))
    {
      QString name = colormap.attribute("name");
      ColorMapLibrary()[name]->setScrip(colormap.text());
    }
  }

  QByteArray snippets_saved_xml =
      settings.value("AddCustomPlotDialog.savedXML", QByteArray()).toByteArray();

  auto snippets_element = root.firstChildElement("snippets");
  if (!snippets_element.isNull())
  {
    auto snippets_previous = GetSnippetsFromXML(snippets_saved_xml);
    auto snippets_layout = GetSnippetsFromXML(snippets_element);

    bool snippets_are_different = false;
    for (const auto& snippet_it : snippets_layout)
    {
      auto prev_it = snippets_previous.find(snippet_it.first);

      if (prev_it == snippets_previous.end() ||
          prev_it->second.function != snippet_it.second.function ||
          prev_it->second.global_vars != snippet_it.second.global_vars)
      {
        snippets_are_different = true;
        break;
      }
    }

    if (snippets_are_different)
    {
      QMessageBox msgBox(this);
      msgBox.setWindowTitle("Overwrite custom transforms?");
      msgBox.setText("Your layout file contains a set of custom transforms different "
                     "from "
                     "the last one you used.\nWant to load these transformations?");
      msgBox.addButton(QMessageBox::No);
      msgBox.addButton(QMessageBox::Yes);
      msgBox.setDefaultButton(QMessageBox::Yes);

      if (msgBox.exec() == QMessageBox::Yes)
      {
        for (const auto& snippet_it : snippets_layout)
        {
          snippets_previous[snippet_it.first] = snippet_it.second;
        }
        QDomDocument doc;
        auto snippets_root_element = ExportSnippets(snippets_previous, doc);
        doc.appendChild(snippets_root_element);
        settings.setValue("AddCustomPlotDialog.savedXML", doc.toByteArray(2));
      }
    }
  }

  ///--------------------------------------------------

  xmlLoadState(domDocument);

  linkedZoomOut();

  _undo_states.clear();
  _undo_states.push_back(domDocument);
  return true;
}

void MainWindow::linkedZoomOut()
{
  if (ui->buttonLink->isChecked())
  {
    for (const auto& it : TabbedPlotWidget::instances())
    {
      auto tabs = it.second->tabWidget();
      for (int t = 0; t < tabs->count(); t++)
      {
        if (PlotDocker* matrix = dynamic_cast<PlotDocker*>(tabs->widget(t)))
        {
          bool first = true;
          Range range;
          // find the ideal zoom
          for (int index = 0; index < matrix->plotCount(); index++)
          {
            PlotWidget* plot = matrix->plotAt(index);
            if (plot->isEmpty())
            {
              continue;
            }

            auto rect = plot->maxZoomRect();
            if (first)
            {
              range.min = rect.left();
              range.max = rect.right();
              first = false;
            }
            else
            {
              range.min = std::min(rect.left(), range.min);
              range.max = std::max(rect.right(), range.max);
            }
          }

          for (int index = 0; index < matrix->plotCount() && !first; index++)
          {
            PlotWidget* plot = matrix->plotAt(index);
            if (plot->isEmpty())
            {
              continue;
            }
            QRectF bound_act = plot->maxZoomRect();
            bound_act.setLeft(range.min);
            bound_act.setRight(range.max);
            plot->setZoomRectangle(bound_act, false);
            plot->replot();
          }
        }
      }
    }
  }
  else
  {
    this->forEachWidget([](PlotWidget* plot) { plot->zoomOut(false); });
  }
}

void MainWindow::on_tabbedAreaDestroyed(QObject* object)
{
  this->setFocus();
}

void MainWindow::forEachWidget(
    std::function<void(PlotWidget*, PlotDocker*, int)> operation)
{
  auto func = [&](QTabWidget* tabs) {
    for (int t = 0; t < tabs->count(); t++)
    {
      PlotDocker* matrix = dynamic_cast<PlotDocker*>(tabs->widget(t));
      if (!matrix)
      {
        continue;
      }

      for (int index = 0; index < matrix->plotCount(); index++)
      {
        PlotWidget* plot = matrix->plotAt(index);
        operation(plot, matrix, index);
      }
    }
  };

  for (const auto& it : TabbedPlotWidget::instances())
  {
    func(it.second->tabWidget());
  }
}

void MainWindow::forEachWidget(std::function<void(PlotWidget*)> op)
{
  forEachWidget([&](PlotWidget* plot, PlotDocker*, int) { op(plot); });
}

void MainWindow::updateTimeSlider()
{
  auto range = calculateVisibleRangeX();

  ui->timeSlider->setLimits(std::get<0>(range), std::get<1>(range), std::get<2>(range));

  _tracker_time = std::max(_tracker_time, ui->timeSlider->getMinimum());
  _tracker_time = std::min(_tracker_time, ui->timeSlider->getMaximum());
}

void MainWindow::updateTimeOffset()
{
  auto range = calculateVisibleRangeX();
  double min_time = std::get<0>(range);

  const bool remove_offset = ui->buttonRemoveTimeOffset->isChecked();
  if (remove_offset && min_time != std::numeric_limits<double>::max())
  {
    _time_offset.set(min_time);
  }
  else
  {
    _time_offset.set(0.0);
  }
}

void MainWindow::updateDataAndReplot(bool replot_hidden_tabs)
{
  _replot_timer->stop();

  MoveDataRet move_ret;

  if (_active_streamer_plugin)
  {
    {
      std::lock_guard<std::mutex> lock(_active_streamer_plugin->mutex());
      move_ret = MoveData(_active_streamer_plugin->dataMap(), _mapped_plot_data, false);
    }

    for (const auto& str : move_ret.added_curves)
    {
      _curvelist_widget->addCurve(str);
    }

    if (move_ret.curves_updated)
    {
      _curvelist_widget->refreshColumns();
    }

    if (ui->streamingSpinBox->value() == ui->streamingSpinBox->maximum())
    {
      _mapped_plot_data.setMaximumRangeX(std::numeric_limits<double>::max());
    }
    else
    {
      _mapped_plot_data.setMaximumRangeX(ui->streamingSpinBox->value());
    }
  }

  const bool is_streaming_active = isStreamingActive();

  //--------------------------------
  std::vector<TransformFunction*> transforms;
  transforms.reserve(_transform_functions.size());
  for (auto& [id, function] : _transform_functions)
  {
    transforms.push_back(function.get());
  }
  std::sort(
      transforms.begin(), transforms.end(),
      [](TransformFunction* a, TransformFunction* b) { return a->order() < b->order(); });

  // Update the reactive plots
  updateReactivePlots();

  // update all transforms, but not the ReactiveLuaFunction
  for (auto& function : transforms)
  {
    if (dynamic_cast<ReactiveLuaFunction*>(function) == nullptr)
    {
      function->calculate();
    }
  }

  forEachWidget([](PlotWidget* plot) { plot->updateCurves(false); });

  //--------------------------------
  // trigger again the execution of this callback if steaming == true
  if (is_streaming_active)
  {
    auto range = calculateVisibleRangeX();
    double max_time = std::get<1>(range);
    _tracker_time = max_time;
    onTrackerTimeUpdated(_tracker_time, false);
  }
  else
  {
    updateTimeOffset();
    updateTimeSlider();
  }
  //--------------------------------
  linkedZoomOut();
}

void MainWindow::on_streamingSpinBox_valueChanged(int value)
{
  double real_value = value;

  if (value == ui->streamingSpinBox->maximum())
  {
    real_value = std::numeric_limits<double>::max();
    ui->streamingSpinBox->setStyleSheet("QSpinBox { color: red; }");
    ui->streamingSpinBox->setSuffix("=inf");
  }
  else
  {
    ui->streamingSpinBox->setStyleSheet("QSpinBox { color: black; }");
    ui->streamingSpinBox->setSuffix(" sec");
  }

  if (isStreamingActive() == false)
  {
    return;
  }

  _mapped_plot_data.setMaximumRangeX(real_value);

  if (_active_streamer_plugin)
  {
    _active_streamer_plugin->setMaximumRangeX(real_value);
  }
}

void MainWindow::on_actionExit_triggered()
{
  this->close();
}

void MainWindow::on_buttonRemoveTimeOffset_toggled(bool)
{
  updateTimeOffset();
  updatedDisplayTime();

  forEachWidget([](PlotWidget* plot) { plot->replot(); });

  if (this->signalsBlocked() == false)
  {
    onUndoableChange();
  }
}

void MainWindow::updatedDisplayTime()
{
  QLineEdit* timeLine = ui->displayTime;
  const double relative_time = _tracker_time - _time_offset.get();
  if (ui->buttonUseDateTime->isChecked())
  {
    if (ui->buttonRemoveTimeOffset->isChecked())
    {
      QTime time = QTime::fromMSecsSinceStartOfDay(std::round(relative_time * 1000.0));
      timeLine->setText(time.toString("HH:mm::ss.zzz"));
    }
    else
    {
      QDateTime datetime =
          QDateTime::fromMSecsSinceEpoch(std::round(_tracker_time * 1000.0));
      timeLine->setText(datetime.toString("[yyyy MMM dd] HH:mm::ss.zzz"));
    }
  }
  else
  {
    timeLine->setText(QString::number(relative_time, 'f', 3));
  }

  QFontMetrics fm(timeLine->font());
  int width = fm.width(timeLine->text()) + 10;
  timeLine->setFixedWidth(std::max(100, width));
}

void MainWindow::on_buttonActivateGrid_toggled(bool checked)
{
  forEachWidget([checked](PlotWidget* plot) {
    plot->activateGrid(checked);
    plot->replot();
  });
}

void MainWindow::on_buttonRatio_toggled(bool checked)
{
  forEachWidget([checked](PlotWidget* plot) {
    plot->setKeepRatioXY(checked);
    plot->replot();
  });
}

void MainWindow::on_buttonPlay_toggled(bool checked)
{
  if (checked)
  {
    _publish_timer->start();
    _prev_publish_time = QDateTime::currentDateTime();
  }
  else
  {
    _publish_timer->stop();
  }
}

void MainWindow::on_actionClearBuffer_triggered()
{
  for (auto& it : _mapped_plot_data.numeric)
  {
    it.second.clear();
  }

  for (auto& it : _mapped_plot_data.strings)
  {
    it.second.clear();
  }

  for (auto& it : _mapped_plot_data.user_defined)
  {
    it.second.clear();
  }

  for (auto& it : _transform_functions)
  {
    it.second->reset();
  }

  forEachWidget([](PlotWidget* plot) {
    plot->reloadPlotData();
    plot->replot();
  });
}

void MainWindow::on_deleteSerieFromGroup(std::string group_name)
{
  std::vector<std::string> names;

  auto AddFromGroup = [&](auto& series) {
    for (auto& it : series)
    {
      const auto& group = it.second.group();
      if (group && group->name() == group_name)
      {
        names.push_back(it.first);
      }
    }
  };
  AddFromGroup(_mapped_plot_data.numeric);
  AddFromGroup(_mapped_plot_data.strings);
  AddFromGroup(_mapped_plot_data.user_defined);

  onDeleteMultipleCurves(names);
}

void MainWindow::on_streamingNotificationsChanged(int active_count)
{
  if (active_count > 0 && _active_streamer_plugin)
  {
    enableStreamingNotificationsButton(true);

    QString tooltipText = QString("%1 has %2 outstanding notitication%3")
                              .arg(_active_streamer_plugin->name())
                              .arg(active_count)
                              .arg(active_count > 1 ? "s" : "");
    ui->buttonStreamingNotifications->setToolTip(tooltipText);
  }
  else
  {
    enableStreamingNotificationsButton(false);
    ui->buttonStreamingNotifications->setToolTip("View streaming alerts");
  }
}

void MainWindow::on_buttonUseDateTime_toggled(bool checked)
{
  static bool first = true;
  if (checked && ui->buttonRemoveTimeOffset->isChecked())
  {
    if (first)
    {
      QMessageBox::information(this, tr("Note"),
                               tr("When \"Use Date Time\" is checked, the option "
                                  "\"Remove Time Offset\" "
                                  "is automatically disabled.\n"
                                  "This message will be shown only once."));
      first = false;
    }
    ui->buttonRemoveTimeOffset->setChecked(false);
  }
  updatedDisplayTime();
}

void MainWindow::on_buttonTimeTracker_pressed()
{
  if (_tracker_param == CurveTracker::LINE_ONLY)
  {
    _tracker_param = CurveTracker::VALUE;
  }
  else if (_tracker_param == CurveTracker::VALUE)
  {
    _tracker_param = CurveTracker::VALUE_NAME;
  }
  else if (_tracker_param == CurveTracker::VALUE_NAME)
  {
    _tracker_param = CurveTracker::LINE_ONLY;
  }
  ui->buttonTimeTracker->setIcon(_tracker_button_icons[_tracker_param]);

  forEachWidget([&](PlotWidget* plot) {
    plot->configureTracker(_tracker_param);
    plot->replot();
  });
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  _replot_timer->stop();
  _publish_timer->stop();

  if (_active_streamer_plugin)
  {
    _active_streamer_plugin->shutdown();
    _active_streamer_plugin = nullptr;
  }
  QSettings settings;
  settings.setValue("MainWindow.geometry", saveGeometry());
  settings.setValue("MainWindow.state", saveState());

  settings.setValue("MainWindow.activateGrid", ui->buttonActivateGrid->isChecked());
  settings.setValue("MainWindow.removeTimeOffset",
                    ui->buttonRemoveTimeOffset->isChecked());
  settings.setValue("MainWindow.dateTimeDisplay", ui->buttonUseDateTime->isChecked());
  settings.setValue("MainWindow.buttonLink", ui->buttonLink->isChecked());
  settings.setValue("MainWindow.buttonRatio", ui->buttonRatio->isChecked());

  settings.setValue("MainWindow.streamingBufferValue", ui->streamingSpinBox->value());
  settings.setValue("MainWindow.timeTrackerSetting", (int)_tracker_param);
  settings.setValue("MainWindow.splitterWidth", ui->mainSplitter->sizes()[0]);

  _data_loader.clear();
  _data_streamer.clear();
  _state_publisher.clear();
  _toolboxes.clear();
}

void MainWindow::onAddCustomPlot(const std::string& plot_name)
{
  ui->widgetStack->setCurrentIndex(1);
  _function_editor->setLinkedPlotName(QString::fromStdString(plot_name));
  _function_editor->createNewPlot();
}

void MainWindow::onEditCustomPlot(const std::string& plot_name)
{
  ui->widgetStack->setCurrentIndex(1);
  auto custom_it = _transform_functions.find(plot_name);
  if (custom_it == _transform_functions.end())
  {
    qWarning("failed to find custom equation");
    return;
  }
  _function_editor->editExistingPlot(
      std::dynamic_pointer_cast<LuaCustomFunction>(custom_it->second));
}

void MainWindow::onRefreshCustomPlot(const std::string& plot_name)
{
  try
  {
    auto custom_it = _transform_functions.find(plot_name);
    if (custom_it == _transform_functions.end())
    {
      qWarning("failed to find custom equation");
      return;
    }
    CustomPlotPtr ce = std::dynamic_pointer_cast<LuaCustomFunction>(custom_it->second);
    ce->calculateAndAdd(_mapped_plot_data);

    onUpdateLeftTableValues();
    updateDataAndReplot(true);
  }
  catch (const std::runtime_error& e)
  {
    QMessageBox::critical(this, "error",
                          "Failed to refresh data : " + QString::fromStdString(e.what()));
  }
}

void MainWindow::onPlaybackLoop()
{
  qint64 delta_ms =
      (QDateTime::currentMSecsSinceEpoch() - _prev_publish_time.toMSecsSinceEpoch());
  _prev_publish_time = QDateTime::currentDateTime();
  delta_ms = std::max((qint64)_publish_timer->interval(), delta_ms);

  _tracker_time += delta_ms * 0.001 * ui->playbackRate->value();
  if (_tracker_time >= ui->timeSlider->getMaximum())
  {
    if (!ui->playbackLoop->isChecked())
    {
      ui->buttonPlay->setChecked(false);
    }
    _tracker_time = ui->timeSlider->getMinimum();
  }
  //////////////////
  auto prev = ui->timeSlider->blockSignals(true);
  ui->timeSlider->setRealValue(_tracker_time);
  ui->timeSlider->blockSignals(prev);

  //////////////////
  updatedDisplayTime();
  onUpdateLeftTableValues();
  updateReactivePlots();

  for (auto& it : _state_publisher)
  {
    it.second->play(_tracker_time);
  }

  forEachWidget([&](PlotWidget* plot) {
    plot->setTrackerPosition(_tracker_time);
    plot->replot();
  });
}

void MainWindow::onCustomPlotCreated(std::vector<CustomPlotPtr> custom_plots)
{
  std::set<PlotWidget*> widget_to_replot;

  for (auto custom_plot : custom_plots)
  {
    const std::string& curve_name = custom_plot->aliasName().toStdString();
    // clear already existing data first
    auto data_it = _mapped_plot_data.numeric.find(curve_name);
    if (data_it != _mapped_plot_data.numeric.end())
    {
      data_it->second.clear();
    }
    try
    {
      custom_plot->calculateAndAdd(_mapped_plot_data);
    }
    catch (std::exception& ex)
    {
      QMessageBox::warning(this, tr("Warning"),
                           tr("Failed to create the custom timeseries. "
                              "Error:\n\n%1")
                               .arg(ex.what()));
    }

    // keep data for reference
    auto custom_it = _transform_functions.find(curve_name);
    if (custom_it == _transform_functions.end())
    {
      _transform_functions.insert({ curve_name, custom_plot });
      _curvelist_widget->addCustom(QString::fromStdString(curve_name));
    }
    else
    {
      custom_it->second = custom_plot;
    }

    forEachWidget([&](PlotWidget* plot) {
      if (plot->curveFromTitle(QString::fromStdString(curve_name)))
      {
        widget_to_replot.insert(plot);
      }
    });
  }

  onUpdateLeftTableValues();
  ui->widgetStack->setCurrentIndex(0);
  _function_editor->clear();

  for (auto plot : widget_to_replot)
  {
    plot->updateCurves(true);
    plot->replot();
  }
  _curvelist_widget->clearSelections();
}

void MainWindow::on_actionReportBug_triggered()
{
  QDesktopServices::openUrl(QUrl("https://github.com/facontidavide/PlotJuggler/issues"));
}

void MainWindow::on_actionShare_the_love_triggered()
{
  QDesktopServices::openUrl(QUrl("https://twitter.com/intent/"
                                 "tweet?hashtags=PlotJuggler"));
}

void MainWindow::on_actionAbout_triggered()
{
  QDialog* dialog = new QDialog(this);
  auto ui = new Ui::AboutDialog();
  ui->setupUi(dialog);

  ui->label_version->setText(QString("version: ") + QApplication::applicationVersion());
  dialog->setAttribute(Qt::WA_DeleteOnClose);

  QFile fileTitle(_skin_path + "/about_window_title.html");
  if (fileTitle.open(QIODevice::ReadOnly))
  {
    ui->titleTextBrowser->setHtml(fileTitle.readAll());
  }

  QFile fileBody(_skin_path + "/about_window_body.html");
  if (fileBody.open(QIODevice::ReadOnly))
  {
    ui->bodyTextBrowser->setHtml(fileBody.readAll());
  }

  dialog->setAttribute(Qt::WA_DeleteOnClose);
  dialog->exec();
}

void MainWindow::on_actionCheatsheet_triggered()
{
  QSettings settings;

  CheatsheetDialog* dialog = new CheatsheetDialog(this);
  dialog->restoreGeometry(settings.value("Cheatsheet.geometry").toByteArray());
  dialog->exec();
  settings.setValue("Cheatsheet.geometry", dialog->saveGeometry());
  dialog->deleteLater();
}

void MainWindow::on_actionSupportPlotJuggler_triggered()
{
  QDialog* dialog = new QDialog(this);
  auto ui = new Ui::SupportDialog();
  ui->setupUi(dialog);

  dialog->setAttribute(Qt::WA_DeleteOnClose);

  dialog->exec();
}

/*
void MainWindow::on_actionSaveAllPlotTabs_triggered()
{
  QSettings settings;
  QString directory_path = settings.value("MainWindow.saveAllPlotTabs",
QDir::currentPath()).toString();
  // Get destination folder
  QFileDialog saveDialog(this);
  saveDialog.setDirectory(directory_path);
  saveDialog.setFileMode(QFileDialog::FileMode::Directory);
  saveDialog.setAcceptMode(QFileDialog::AcceptSave);
  saveDialog.exec();

  uint image_number = 1;
  if (saveDialog.result() == QDialog::Accepted && !saveDialog.selectedFiles().empty())
  {
    // Save Plots
    QString directory = saveDialog.selectedFiles().first();
    settings.setValue("MainWindow.saveAllPlotTabs", directory);

    QStringList file_names;
    QStringList existing_files;
    QDateTime current_date_time(QDateTime::currentDateTime());
    QString current_date_time_name(current_date_time.toString("yyyy-MM-dd_HH-mm-ss"));
    for (const auto& it : TabbedPlotWidget::instances())
    {
      auto tab_widget = it.second->tabWidget();
      for (int i = 0; i < tab_widget->count(); i++)
      {
        PlotDocker* matrix = static_cast<PlotDocker*>(tab_widget->widget(i));
        QString name = QString("%1/%2_%3_%4.png")
                           .arg(directory)
                           .arg(current_date_time_name)
                           .arg(image_number, 2, 10, QLatin1Char('0'))
                           .arg(matrix->name());
        file_names.push_back(name);
        image_number++;

        QFileInfo check_file(file_names.back());
        if (check_file.exists() && check_file.isFile())
        {
          existing_files.push_back(name);
        }
      }
    }
    if (existing_files.isEmpty() == false)
    {
      QMessageBox msgBox;
      msgBox.setText("One or more files will be overwritten. ant to continue?");
      QString all_files;
      for (const auto& str : existing_files)
      {
        all_files.push_back("\n");
        all_files.append(str);
      }
      msgBox.setInformativeText(all_files);
      msgBox.setStandardButtons(QMessageBox::Cancel | QMessageBox::Ok);
      msgBox.setDefaultButton(QMessageBox::Ok);

      if (msgBox.exec() != QMessageBox::Ok)
      {
        return;
      }
    }

    image_number = 0;
    for (const auto& it : TabbedPlotWidget::instances())
    {
      auto tab_widget = it.second->tabWidget();
      for (int i = 0; i < tab_widget->count(); i++)
      {
        PlotDocker* matrix = static_cast<PlotDocker*>(tab_widget->widget(i));
        TabbedPlotWidget::saveTabImage(file_names[image_number], matrix);
        image_number++;
      }
    }
  }
}*/

void MainWindow::on_buttonLoadDatafile_clicked()
{
  if (_data_loader.empty())
  {
    QMessageBox::warning(this, tr("Warning"),
                         tr("No plugin was loaded to process a data file\n"));
    return;
  }

  QSettings settings;

  QString file_extension_filter;

  std::set<QString> extensions;

  for (auto& it : _data_loader)
  {
    DataLoaderPtr loader = it.second;
    for (QString extension : loader->compatibleFileExtensions())
    {
      extensions.insert(extension.toLower());
    }
  }

  for (const auto& it : extensions)
  {
    file_extension_filter.append(QString(" *.") + it);
  }

  QString directory_path =
      settings.value("MainWindow.lastDatafileDirectory", QDir::currentPath()).toString();

  QFileDialog loadDialog(this);
  loadDialog.setFileMode(QFileDialog::ExistingFiles);
  loadDialog.setViewMode(QFileDialog::Detail);
//  loadDialog.setNameFilter(file_extension_filter);
  loadDialog.setNameFilter("*.log");
  loadDialog.setDirectory(directory_path);

  QStringList fileNames;
  if (loadDialog.exec())
  {
    fileNames = loadDialog.selectedFiles();
  }

  if (fileNames.isEmpty())
  {
    return;
  }

  directory_path = QFileInfo(fileNames[0]).absolutePath();
  settings.setValue("MainWindow.lastDatafileDirectory", directory_path);

  if (loadDataFromFiles(fileNames))
  {
    updateRecentDataMenu(fileNames);
  }
}

void MainWindow::on_buttonLoadLayout_clicked()
{
  QSettings settings;

  QString directory_path =
      settings.value("MainWindow.lastLayoutDirectory", QDir::currentPath()).toString();
  QString filename =
      QFileDialog::getOpenFileName(this, "Open Layout", directory_path, "*.xml");
  if (filename.isEmpty())
  {
    return;
  }

  if (loadLayoutFromFile(filename))
  {
    updateRecentLayoutMenu({ filename });
  }

  directory_path = QFileInfo(filename).absolutePath();
  settings.setValue("MainWindow.lastLayoutDirectory", directory_path);
}

void MainWindow::on_buttonSaveLayout_clicked()
{
  QDomDocument doc = xmlSaveState();

  QSettings settings;

  QString directory_path =
      settings.value("MainWindow.lastLayoutDirectory", QDir::currentPath()).toString();

  QFileDialog saveDialog(this);
  saveDialog.setOption(QFileDialog::DontUseNativeDialog, true);

  QGridLayout* save_layout = static_cast<QGridLayout*>(saveDialog.layout());

  QFrame* frame = new QFrame;
  frame->setFrameStyle(QFrame::Box | QFrame::Plain);
  frame->setLineWidth(1);

  QVBoxLayout* vbox = new QVBoxLayout;
  QLabel* title = new QLabel("Save Layout options");
  QFrame* separator = new QFrame;
  separator->setFrameStyle(QFrame::HLine | QFrame::Plain);

  auto checkbox_datasource = new QCheckBox("Save data source");
  checkbox_datasource->setToolTip("the layout will remember the source of your data,\n"
                                  "i.e. the Datafile used or the Streaming Plugin loaded "
                                  "?");
  checkbox_datasource->setFocusPolicy(Qt::NoFocus);
  checkbox_datasource->setChecked(
      settings.value("MainWindow.saveLayoutDataSource", true).toBool());

  auto checkbox_snippets = new QCheckBox("Save Scripts (transforms and colormaps)");
  checkbox_snippets->setToolTip("Do you want the layout to save your Lua scripts?");
  checkbox_snippets->setFocusPolicy(Qt::NoFocus);
  checkbox_snippets->setChecked(
      settings.value("MainWindow.saveLayoutSnippets", true).toBool());

  vbox->addWidget(title);
  vbox->addWidget(separator);
  vbox->addWidget(checkbox_datasource);
  vbox->addWidget(checkbox_snippets);
  frame->setLayout(vbox);

  int rows = save_layout->rowCount();
  int col = save_layout->columnCount();
  save_layout->addWidget(frame, 0, col, rows, 1, Qt::AlignTop);

  saveDialog.setAcceptMode(QFileDialog::AcceptSave);
  saveDialog.setDefaultSuffix("xml");
  saveDialog.setNameFilter("XML (*.xml)");
  saveDialog.setDirectory(directory_path);
  saveDialog.exec();

  if (saveDialog.result() != QDialog::Accepted || saveDialog.selectedFiles().empty())
  {
    return;
  }

  QString fileName = saveDialog.selectedFiles().first();

  if (fileName.isEmpty())
  {
    return;
  }

  directory_path = QFileInfo(fileName).absolutePath();
  settings.setValue("MainWindow.lastLayoutDirectory", directory_path);
  settings.setValue("MainWindow.saveLayoutDataSource", checkbox_datasource->isChecked());
  settings.setValue("MainWindow.saveLayoutSnippets", checkbox_snippets->isChecked());

  QDomElement root = doc.namedItem("root").toElement();

  root.appendChild(doc.createComment(" - - - - - - - - - - - - - - "));

  root.appendChild(doc.createComment(" - - - - - - - - - - - - - - "));

  root.appendChild(savePluginState(doc));

  root.appendChild(doc.createComment(" - - - - - - - - - - - - - - "));

  if (checkbox_datasource->isChecked())
  {
    QDomElement loaded_list = doc.createElement("previouslyLoaded_Datafiles");

    for (const auto& loaded : _loaded_datafiles_history)
    {
      QString loaded_datafile = QDir(directory_path).relativeFilePath(loaded.filename);

      QDomElement file_elem = doc.createElement("fileInfo");
      file_elem.setAttribute("filename", loaded_datafile);
      file_elem.setAttribute("prefix", loaded.prefix);

      file_elem.appendChild(loaded.plugin_config.firstChild());
      loaded_list.appendChild(file_elem);
    }
    root.appendChild(loaded_list);

    if (_active_streamer_plugin)
    {
      QDomElement loaded_streamer = doc.createElement("previouslyLoaded_Streamer");
      QString streamer_name = _active_streamer_plugin->name();
      loaded_streamer.setAttribute("name", streamer_name);
      root.appendChild(loaded_streamer);
    }
  }
  //-----------------------------------
  root.appendChild(doc.createComment(" - - - - - - - - - - - - - - "));
  if (checkbox_snippets->isChecked())
  {
    QDomElement custom_equations = doc.createElement("customMathEquations");
    for (const auto& custom_it : _transform_functions)
    {
      const auto& custom_plot = custom_it.second;
      custom_plot->xmlSaveState(doc, custom_equations);
    }
    root.appendChild(custom_equations);

    QByteArray snippets_xml_text =
        settings.value("AddCustomPlotDialog.savedXML", QByteArray()).toByteArray();
    auto snipped_saved = GetSnippetsFromXML(snippets_xml_text);
    auto snippets_root = ExportSnippets(snipped_saved, doc);
    root.appendChild(snippets_root);

    QDomElement color_maps = doc.createElement("colorMaps");
    for (const auto& it : ColorMapLibrary())
    {
      QString colormap_name = it.first;
      QDomElement colormap = doc.createElement("colorMap");
      QDomText colormap_script = doc.createTextNode(it.second->script());
      colormap.setAttribute("name", colormap_name);
      colormap.appendChild(colormap_script);
      color_maps.appendChild(colormap);
    }
  }
  root.appendChild(doc.createComment(" - - - - - - - - - - - - - - "));
  //------------------------------------
  QFile file(fileName);
  if (file.open(QIODevice::WriteOnly))
  {
    QTextStream stream(&file);
    stream << doc.toString() << "\n";
  }
}

void MainWindow::onActionFullscreenTriggered()
{
  static bool first_call = true;
  if (first_call && !_minimized)
  {
    first_call = false;
    QMessageBox::information(this, "Remember!",
                             "Press F10 to switch back to the normal view");
  }

  _minimized = !_minimized;

  ui->leftMainWindowFrame->setVisible(!_minimized);
  ui->widgetTimescale->setVisible(!_minimized);
  ui->menuBar->setVisible(!_minimized);

  for (auto& it : TabbedPlotWidget::instances())
  {
    it.second->setControlsVisible(!_minimized);
  }
}

void MainWindow::on_actionClearRecentData_triggered()
{
  QMenu* menu = _recent_data_files;
  for (QAction* action : menu->actions())
  {
    if (action->isSeparator())
    {
      break;
    }
    menu->removeAction(action);
  }
  menu->setEnabled(false);
  QSettings settings;
  settings.setValue("MainWindow.recentlyLoadedDatafile", {});
}

void MainWindow::on_actionClearRecentLayout_triggered()
{
  QMenu* menu = _recent_layout_files;
  for (QAction* action : menu->actions())
  {
    if (action->isSeparator())
    {
      break;
    }
    menu->removeAction(action);
  }
  menu->setEnabled(false);
  QSettings settings;
  settings.setValue("MainWindow.recentlyLoadedLayout", {});
}

void MainWindow::on_actionDeleteAllData_triggered()
{
  QMessageBox msgBox(this);
  msgBox.setWindowTitle("Warning. Can't be undone.");
  msgBox.setText(tr("Do you want to remove the previously loaded data?\n"));
  msgBox.addButton(QMessageBox::No);
  msgBox.addButton(QMessageBox::Yes);
  msgBox.setDefaultButton(QMessageBox::Yes);
  auto reply = msgBox.exec();

  if (reply == QMessageBox::No)
  {
    return;
  }

  deleteAllData();
}

void MainWindow::on_actionPreferences_triggered()
{
  QSettings settings;
  QString prev_style = settings.value("Preferences::theme", "light").toString();

  PreferencesDialog dialog;
  dialog.exec();

  QString theme = settings.value("Preferences::theme").toString();

  if (!theme.isEmpty() && theme != prev_style)
  {
    loadStyleSheet(tr(":/resources/stylesheet_%1.qss").arg(theme));
  }
}

void MainWindow::on_playbackStep_valueChanged(double step)
{
  ui->timeSlider->setFocus();
  ui->timeSlider->setRealStepValue(step);
}

void MainWindow::on_actionLoadStyleSheet_triggered()
{
  QSettings settings;
  QString directory_path =
      settings.value("MainWindow.loadStyleSheetDirectory", QDir::currentPath())
          .toString();

  QString fileName = QFileDialog::getOpenFileName(this, tr("Load StyleSheet"),
                                                  directory_path, tr("(*.qss)"));
  if (fileName.isEmpty())
  {
    return;
  }

  loadStyleSheet(fileName);

  directory_path = QFileInfo(fileName).absolutePath();
  settings.setValue("MainWindow.loadStyleSheetDirectory", directory_path);
}

void MainWindow::on_buttonLegend_clicked()
{
  switch (_labels_status)
  {
    case LabelStatus::LEFT:
      _labels_status = LabelStatus::HIDDEN;
      break;
    case LabelStatus::RIGHT:
      _labels_status = LabelStatus::LEFT;
      break;
    case LabelStatus::HIDDEN:
      _labels_status = LabelStatus::RIGHT;
      break;
  }

  auto visitor = [=](PlotWidget* plot) {
    plot->activateLegend(_labels_status != LabelStatus::HIDDEN);

    if (_labels_status == LabelStatus::LEFT)
    {
      plot->setLegendAlignment(Qt::AlignLeft);
    }
    else if (_labels_status == LabelStatus::RIGHT)
    {
      plot->setLegendAlignment(Qt::AlignRight);
    }
    plot->replot();
  };

  this->forEachWidget(visitor);
}

void MainWindow::on_buttonZoomOut_clicked()
{
  linkedZoomOut();
  onUndoableChange();
}

void MainWindow::on_comboStreaming_currentIndexChanged(const QString& current_text)
{
  QSettings settings;
  settings.setValue("MainWindow.previousStreamingPlugin", current_text);
  auto streamer = _data_streamer.at(current_text);
  ui->buttonStreamingOptions->setEnabled(!streamer->availableActions().empty());

  std::pair<QAction*, int> notifications_pair = streamer->notificationAction();
  if (notifications_pair.first == nullptr)
  {
    ui->buttonStreamingNotifications->setEnabled(false);
  }
  else
  {
    on_streamingNotificationsChanged(notifications_pair.second);
  }
}

void MainWindow::on_buttonStreamingStart_clicked()
{
  ui->buttonStreamingStart->setEnabled(false);
  if (ui->buttonStreamingStart->text() == "Start")
  {
    startStreamingPlugin(ui->comboStreaming->currentText());
  }
  else
  {
    stopStreamingPlugin();
  }
  ui->buttonStreamingStart->setEnabled(true);
}

PopupMenu::PopupMenu(QWidget* relative_widget, QWidget* parent)
  : QMenu(parent), _w(relative_widget)
{
}

void PopupMenu::showEvent(QShowEvent*)
{
  QPoint p = _w->mapToGlobal({});
  QRect geo = _w->geometry();
  this->move(p.x() + geo.width(), p.y());
}

void PopupMenu::leaveEvent(QEvent*)
{
  close();
}

void PopupMenu::closeEvent(QCloseEvent*)
{
  _w->setAttribute(Qt::WA_UnderMouse, false);
}

void MainWindow::on_buttonRecentData_clicked()
{
  PopupMenu* menu = new PopupMenu(ui->buttonRecentData, this);

  for (auto action : _recent_data_files->actions())
  {
    menu->addAction(action);
  }
  menu->exec();
}

void MainWindow::on_buttonStreamingOptions_clicked()
{
  if (_data_streamer.empty())
  {
    return;
  }
  auto streamer = _data_streamer.at(ui->comboStreaming->currentText());

  PopupMenu* menu = new PopupMenu(ui->buttonStreamingOptions, this);
  for (auto action : streamer->availableActions())
  {
    menu->addAction(action);
  }
  menu->show();
}

void MainWindow::on_buttonHideFileFrame_clicked()
{
  bool hidden = !ui->frameFile->isHidden();
  ui->buttonHideFileFrame->setText(hidden ? "+" : " -");
  ui->frameFile->setHidden(hidden);

  QSettings settings;
  settings.setValue("MainWindow.hiddenFileFrame", hidden);
}

void MainWindow::on_buttonHideStreamingFrame_clicked()
{
  bool hidden = !ui->frameStreaming->isHidden();
  ui->buttonHideStreamingFrame->setText(hidden ? "+" : " -");
  ui->frameStreaming->setHidden(hidden);

  QSettings settings;
  settings.setValue("MainWindow.hiddenStreamingFrame", hidden);
}

void MainWindow::on_buttonHidePublishersFrame_clicked()
{
  bool hidden = !ui->framePublishers->isHidden();
  ui->buttonHidePublishersFrame->setText(hidden ? "+" : " -");
  ui->framePublishers->setHidden(hidden);

  QSettings settings;
  settings.setValue("MainWindow.hiddenPublishersFrame", hidden);
}

void MainWindow::on_buttonRecentLayout_clicked()
{
  PopupMenu* menu = new PopupMenu(ui->buttonRecentLayout, this);

  for (auto action : _recent_layout_files->actions())
  {
    menu->addAction(action);
  }
  menu->exec();
}

QStringList MainWindow::readAllCurvesFromXML(QDomElement root_node)
{
  QStringList curves;

  QStringList level_names = { "tabbed_widget", "Tab",  "Container", "DockSplitter",
                              "DockArea",      "plot", "curve" };

  std::function<void(int, QDomElement)> recursiveXmlStream;
  recursiveXmlStream = [&](int level, QDomElement parent_elem) {
    QString level_name = level_names[level];
    for (auto elem = parent_elem.firstChildElement(level_name); elem.isNull() == false;
         elem = elem.nextSiblingElement(level_name))
    {
      if (level_name == "curve")
      {
        curves.push_back(elem.attribute("name"));
      }
      else
      {
        recursiveXmlStream(level + 1, elem);
      }
    }
  };

  // start recursion
  recursiveXmlStream(0, root_node);

  return curves;
}

void MainWindow::on_actionColorMap_Editor_triggered()
{
  ColorMapEditor dialog;
  dialog.exec();
}

void MainWindow::on_buttonReloadData_clicked()
{
  const auto prev_infos = std::move(_loaded_datafiles_previous);
  _loaded_datafiles_previous.clear();
  for (const auto& info : prev_infos)
  {
    loadDataFromFile(info);
  }
  ui->buttonReloadData->setEnabled(!_loaded_datafiles_previous.empty());
}

void MainWindow::on_buttonCloseStatus_clicked()
{
  ui->widgetStatusBar->hide();
}
