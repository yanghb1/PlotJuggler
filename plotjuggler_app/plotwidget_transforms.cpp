/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "plotwidget_transforms.h"
#include "ui_plotwidget_transforms.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSettings>
#include <QListWidgetItem>
#include <QLabel>
#include <QMouseEvent>
#include <QPushButton>
#include <QDialogButtonBox>
#include <QDebug>
#include <qwt_text.h>

DialogTransformEditor::DialogTransformEditor(PlotWidget* plotwidget)
  : QDialog(plotwidget), ui(new Ui::plotwidget_transforms), _plotwidget_origin(plotwidget)
{
  ui->setupUi(this);

  QDomDocument doc;
  auto saved_state = plotwidget->xmlSaveState(doc);
  _plotwidget = new PlotWidget(plotwidget->datamap(), this);
  _plotwidget->on_changeTimeOffset(plotwidget->timeOffset());
  _plotwidget->xmlLoadState(saved_state);

  auto layout = new QVBoxLayout();
  ui->framePlotPreview->setLayout(layout);
  layout->addWidget(_plotwidget);
  layout->setMargin(6);

  _plotwidget->zoomOut(false);
  _plotwidget->setContextMenuEnabled(false);

  setupTable();

  QSettings settings;
  restoreGeometry(settings.value("DialogTransformEditor.geometry").toByteArray());

  ui->listCurves->setStyleSheet("QListView::item:selected { background: #ddeeff; }");

  auto names = TransformFactory::registeredTransforms();

  for (const auto& name : names)
  {
    ui->listTransforms->addItem(QString::fromStdString(name));
  }

  if (ui->listCurves->count() != 0)
  {
    ui->listCurves->item(0)->setSelected(true);
  }
}

void DialogTransformEditor::setupTable()
{
  std::map<QString, QColor> colors = _plotwidget->getCurveColors();

  int row = 0;
  for (auto& it : colors)
  {
    auto curve_it = _plotwidget->curveFromTitle(it.first);
    auto name = QString::fromStdString(curve_it->src_name);
    auto color = it.second;
    auto item = new QListWidgetItem();
    //  item->setForeground(color);
    ui->listCurves->addItem(item);
    auto plot_row = new RowWidget(name, color);
    item->setSizeHint(plot_row->sizeHint());
    ui->listCurves->setItemWidget(item, plot_row);

    row++;
  }
}

DialogTransformEditor::~DialogTransformEditor()
{
  QSettings settings;
  settings.setValue("DialogTransformEditor.geometry", saveGeometry());

  delete ui;
}

DialogTransformEditor::RowWidget::RowWidget(QString text, QColor color)
{
  auto layout = new QHBoxLayout();
  setLayout(layout);
  _text = new QLabel(text, this);

  setStyleSheet(QString("color: %1;").arg(color.name()));
  _color = color;

  layout->addWidget(_text);
}

QString DialogTransformEditor::RowWidget::text() const
{
  return _text->text();
}

QColor DialogTransformEditor::RowWidget::color() const
{
  return _color;
}

void DialogTransformEditor::on_listCurves_itemSelectionChanged()
{
  auto selected = ui->listCurves->selectedItems();
  if (selected.size() != 1)
  {
    return;
  }
  auto item = selected.front();
  auto row_widget = dynamic_cast<RowWidget*>(ui->listCurves->itemWidget(item));
  auto curve_name = row_widget->text();

  auto curve_it = _plotwidget->curveFromTitle(curve_name);
  int transform_row = 0;
  if (auto ts = dynamic_cast<TransformedTimeseries*>(curve_it->curve->data()))
  {
    if (ts->transform())
    {
      for (int row = 1; row < ui->listTransforms->count(); row++)
      {
        if (ui->listTransforms->item(row)->text() == ts->transformName())
        {
          transform_row = row;
          break;
        }
      }
    }
  }

  int selected_row = -1;
  auto selected_transforms = ui->listTransforms->selectedItems();
  if (selected_transforms.size() == 1)
  {
    selected_row = ui->listTransforms->row(selected_transforms.front());
  }

  if (selected_row == transform_row)
  {
    // force callback
    on_listTransforms_itemSelectionChanged();
  }
  else
  {
    ui->listTransforms->item(transform_row)->setSelected(true);
  }
}

void DialogTransformEditor::on_listTransforms_itemSelectionChanged()
{
  auto selected_curves = ui->listCurves->selectedItems();
  if (selected_curves.size() != 1)
  {
    return;
  }
  auto row_widget =
      dynamic_cast<RowWidget*>(ui->listCurves->itemWidget(selected_curves.front()));

  QString curve_name = row_widget->text();

  auto selected_transforms = ui->listTransforms->selectedItems();
  if (selected_transforms.size() != 1)
  {
    return;
  }
  QString transform_ID = selected_transforms.front()->text();

  auto curve_info = _plotwidget->curveFromTitle(curve_name);
  auto qwt_curve = curve_info->curve;
  auto ts = dynamic_cast<TransformedTimeseries*>(curve_info->curve->data());

  QSignalBlocker block(ui->lineEditAlias);

  if (transform_ID.isEmpty() || transform_ID == ui->listTransforms->item(0)->text())
  {
    ts->setTransform({});
    ts->updateCache(true);
    ui->stackedWidgetArguments->setCurrentIndex(0);

    ui->lineEditAlias->setText("");
    ui->lineEditAlias->setEnabled(false);
    qwt_curve->setTitle(curve_name);
  }
  else
  {
    ts->setTransform(transform_ID);
    ts->updateCache(true);
    ui->lineEditAlias->setEnabled(true);

    QString curve_title = qwt_curve->title().text();
    if (ts->alias().isEmpty())
    {
      auto src_name = QString::fromStdString(curve_info->src_name);
      auto new_title = QString("%1[%2]").arg(src_name).arg(transform_ID);
      ts->setAlias(new_title);
    }

    ui->lineEditAlias->setText(ts->alias());
    qwt_curve->setTitle(ts->alias());

    auto widget = ts->transform()->optionsWidget();
    int index = ui->stackedWidgetArguments->indexOf(widget);
    if (index == -1 && widget)
    {
      index = ui->stackedWidgetArguments->addWidget(widget);
    }

    ui->stackedWidgetArguments->setCurrentIndex(index);

    if (_connected_transform_widgets.count(widget) == 0)
    {
      connect(ts->transform().get(), &TransformFunction::parametersChanged, this, [=]() {
        ts->updateCache(true);
        if (ui->checkBoxAutoZoom->isChecked())
        {
          _plotwidget->zoomOut(false);
        }
        else
        {
          _plotwidget->replot();
        }
      });
      _connected_transform_widgets.insert(widget);
    }
  }

  _plotwidget->zoomOut(false);
}

void DialogTransformEditor::on_pushButtonCancel_clicked()
{
  this->reject();
}

void DialogTransformEditor::on_pushButtonSave_clicked()
{
  on_lineEditAlias_editingFinished();

  QSettings settings;
  bool autozoom_filter_applied =
      settings.value("Preferences::autozoom_filter_applied", true).toBool();
  QDomDocument doc;
  auto elem = _plotwidget->xmlSaveState(doc);
  _plotwidget_origin->xmlLoadState(elem, autozoom_filter_applied);

  if (autozoom_filter_applied)
  {
    _plotwidget_origin->zoomOut(false);
  }

  this->accept();
}

void DialogTransformEditor::on_lineEditAlias_editingFinished()
{
  auto selected_curves = ui->listCurves->selectedItems();
  if (selected_curves.size() != 1)
  {
    return;
  }
  auto row_widget =
      dynamic_cast<RowWidget*>(ui->listCurves->itemWidget(selected_curves.front()));

  QString curve_name = row_widget->text();

  auto curve_it = _plotwidget->curveFromTitle(curve_name);
  auto ts = dynamic_cast<TransformedTimeseries*>(curve_it->curve->data());

  curve_it->curve->setTitle(ui->lineEditAlias->text());

  if (ts && ts->transform())
  {
    ts->setAlias(ui->lineEditAlias->text());
  }

  _plotwidget->replot();
}
