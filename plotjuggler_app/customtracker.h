/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef CUSTOMTRACKER_H
#define CUSTOMTRACKER_H

#include <QEvent>
#include <QPointF>
#include "qwt_plot_picker.h"
#include "qwt_picker_machine.h"
#include "qwt_plot_marker.h"

class QwtPlotCurve;

class CurveTracker : public QObject
{
  Q_OBJECT
public:
  explicit CurveTracker(QwtPlot*);

  ~CurveTracker();

  QPointF actualPosition() const;

  typedef enum
  {
    LINE_ONLY,
    VALUE,
    VALUE_NAME
  } Parameter;

public slots:

  void setPosition(const QPointF& pos);

  void setParameter(Parameter par);

  void setEnabled(bool enable);

  bool isEnabled() const;

  void redraw()
  {
    setPosition(_prev_trackerpoint);
  }

private:
  QLineF curveLineAt(const QwtPlotCurve*, double x) const;

  QPointF transform(QPoint);

  QPoint invTransform(QPointF);

  QPointF _prev_trackerpoint;
  std::vector<QwtPlotMarker*> _marker;
  QwtPlotMarker* _line_marker;
  QwtPlotMarker* _text_marker;
  QwtPlot* _plot;
  Parameter _param;
  bool _visible;
};

#endif  // CUSTOMTRACKER_H
