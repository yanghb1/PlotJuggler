/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef PJ_TRANSFORM_FUNCTION_H
#define PJ_TRANSFORM_FUNCTION_H

#include <QApplication>
#include <set>
#include <functional>
#include "PlotJuggler/plotdata.h"
#include "PlotJuggler/pj_plugin.h"

namespace PJ
{
/** @brief Generic interface for a multi input - multi output transformation function.
 * Contrariwise to other plugins, multiple instances of the this class might be created.
 * For this reason, a TransformFactory is also defined
 */
class TransformFunction : public PlotJugglerPlugin
{
  Q_OBJECT

public:
  using Ptr = std::shared_ptr<TransformFunction>;

  TransformFunction();

  virtual ~TransformFunction() = default;

  virtual const char* name() const = 0;

  /** Number of inputs. Return -1 if it is not a constant.
   *
   * When numInputs() > 0, then the data will be initialized using
   * the method:
   *     setDataSource(const std::vector<const PlotData*>& src_data)
   *
   * When  numInputs() == -1, then the number of inputs is undefined and the
   * data will be initialized using the method_
   *     setDataSource( PlotDataMapRef* data )
   */
  virtual int numInputs() const = 0;

  /** Number of outputs. Define the size of the vector used in:
   *     calculate(std::vector<PlotData*>& dst_data)
   */
  virtual int numOutputs() const = 0;

  /** Clear the cache, state and any stored data */
  virtual void reset()
  {
  }

  PlotDataMapRef* plotData()
  {
    return _data;
  }

  std::vector<const PlotData*>& dataSources();

  virtual void setData(PlotDataMapRef* data, const std::vector<const PlotData*>& src_vect,
                       std::vector<PlotData*>& dst_vect);

  virtual void calculate() = 0;

  unsigned order() const
  {
    return _order;
  }

signals:
  void parametersChanged();

protected:
  std::vector<const PlotData*> _src_vector;
  std::vector<PlotData*> _dst_vector;
  PlotDataMapRef* _data;

  unsigned _order;
};

using TransformsMap = std::unordered_map<std::string, std::shared_ptr<TransformFunction>>;

/// Simplified version with Single input and Single output
class TransformFunction_SISO : public TransformFunction
{
  Q_OBJECT
public:
  TransformFunction_SISO() = default;

  void reset() override;

  int numInputs() const override
  {
    return 1;
  }

  int numOutputs() const override
  {
    return 1;
  }

  void calculate() override;

  /// Method to be implemented by the user to apply a statefull function to each point.
  /// Index will increase monotonically, unless reset() is used.
  virtual std::optional<PlotData::Point> calculateNextPoint(size_t index) = 0;

  const PlotData* dataSource() const;

protected:
  double _last_timestamp = std::numeric_limits<double>::lowest();
};

///------ The factory to create instances of a SeriesTransform -------------

class TransformFactory : public QObject
{
public:
  TransformFactory()
  {
  }

private:
  TransformFactory(const TransformFactory&) = delete;
  TransformFactory& operator=(const TransformFactory&) = delete;

  std::map<std::string, std::function<TransformFunction::Ptr()>> creators_;
  std::map<std::string, QDomDocument> default_params_;
  std::set<std::string> names_;

  static TransformFactory* instance();

public:
  static const std::set<std::string>& registeredTransforms();

  template <typename T>
  static void registerTransform()
  {
    const std::string name = T::transformName();
    instance()->names_.insert(name);
    auto doc_it = instance()->default_params_.insert({ name, {} }).first;
    QDomDocument* document = &(doc_it->second);
    instance()->creators_[name] = [document]() {
      auto trans = std::make_shared<T>();
      T* transform = trans.get();

      connect(transform, &TransformFunction_SISO::parametersChanged,
              [transform, document]() {
                *document = {};
                auto root = document->createElement("root");
                document->appendChild(root);
                transform->xmlSaveState(*document, root);
              });
      auto root = document->firstChildElement("root");
      if (!root.isNull())
      {
        QSignalBlocker block(trans.get());
        trans->xmlLoadState(root);
      }
      return trans;
    };
  }

  static TransformFunction::Ptr create(const std::string& name);
};

}  // namespace PJ

Q_DECLARE_OPAQUE_POINTER(PJ::TransformFactory*)
Q_DECLARE_METATYPE(PJ::TransformFactory*)
Q_GLOBAL_STATIC(PJ::TransformFactory, _transform_factory_ptr_from_macro)

QT_BEGIN_NAMESPACE

#define TransformFunction_iid "facontidavide.PlotJuggler3.TransformFunction"
Q_DECLARE_INTERFACE(PJ::TransformFunction, TransformFunction_iid)

#define TransformFunctionSISO_iid "facontidavide.PlotJuggler3.TransformFunctionSISO"
Q_DECLARE_INTERFACE(PJ::TransformFunction_SISO, TransformFunctionSISO_iid)

QT_END_NAMESPACE

#endif
