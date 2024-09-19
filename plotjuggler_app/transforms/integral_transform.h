#ifndef INTEGRAL_TRANSFORM_H
#define INTEGRAL_TRANSFORM_H

#include <QLineEdit>
#include "PlotJuggler/transform_function.h"
#include "ui_integral_transform.h"

using namespace PJ;

class IntegralTransform : public TransformFunction_SISO
{
public:
  IntegralTransform();

  ~IntegralTransform() override;

  static const char* transformName()
  {
    return "Integral";
  }

  const char* name() const override
  {
    return transformName();
  }

  QWidget* optionsWidget() override;

  void reset() override;

  bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const override;

  bool xmlLoadState(const QDomElement& parent_element) override;

  void on_buttonCompute_clicked();

private:
  std::optional<PlotData::Point> calculateNextPoint(size_t index) override;

  QWidget* _widget;
  Ui::IntegralTransform* ui;
  double _dT;

  double _accumulated_value;
};

#endif  // INTEGRAL_TRANSFORM_H
