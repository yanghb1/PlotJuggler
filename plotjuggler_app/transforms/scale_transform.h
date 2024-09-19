#ifndef SCALE_TRANSFORM_H
#define SCALE_TRANSFORM_H

#include <QWidget>
#include "PlotJuggler/transform_function.h"

using namespace PJ;

namespace Ui
{
class ScaleTransform;
}

class ScaleTransform : public TransformFunction_SISO
{
public:
  explicit ScaleTransform();

  ~ScaleTransform() override;

  static const char* transformName()
  {
    return "Scale/Offset";
  }

  const char* name() const override
  {
    return ScaleTransform::transformName();
  }

  QWidget* optionsWidget() override;

  bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const override;

  bool xmlLoadState(const QDomElement& parent_element) override;

private:
  QWidget* _widget;
  Ui::ScaleTransform* ui;

  std::optional<PlotData::Point> calculateNextPoint(size_t index) override;
};

#endif  // SCALE_TRANSFORM_H
