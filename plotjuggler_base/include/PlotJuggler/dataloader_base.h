/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef DATALOAD_TEMPLATE_H
#define DATALOAD_TEMPLATE_H

#include <QFile>

#include "PlotJuggler/plotdata.h"
#include "PlotJuggler/pj_plugin.h"
#include "PlotJuggler/messageparser_base.h"

namespace PJ
{
struct FileLoadInfo
{
  /// name of the file to open
  QString filename;
  /// prefix to be added to the name of the series (optional)
  QString prefix;
  /// Saved configuration from a previous run or a Layout file
  QDomDocument plugin_config;
};

/**
 * @brief The DataLoader plugin type is used to load files.
 *
 * The main application selects the correct plugin using the
 * compatibleFileExtensions()
 */
class DataLoader : public PlotJugglerPlugin
{
public:
  DataLoader() = default;

  virtual ~DataLoader() = default;

  /// Provide a list of file extensions that this plugin can open
  virtual const std::vector<const char*>& compatibleFileExtensions() const = 0;

  virtual bool readDataFromFile(FileLoadInfo* fileload_info,
                                PlotDataMapRef& destination) = 0;

  void loadData(PlotDataMapRef& destination, const std::map<std::string, std::vector<std::pair<double, double>>> &plot_data){

      for(auto iter : plot_data){
          std::vector<PlotData*> plots_vector;

          auto num_it = destination.addNumeric(iter.first);
          plots_vector.push_back(&(num_it->second));
          for(auto value : iter.second){
            plots_vector[0]->pushBack({ value.first, value.second });
          }
      }
  }

  void setParserFactories(ParserFactories* parsers)
  {
    _parser_factories = parsers;
  }

  const ParserFactories* parserFactories() const
  {
    return _parser_factories;
  }

private:
  ParserFactories* _parser_factories = nullptr;
};

using DataLoaderPtr = std::shared_ptr<DataLoader>;

}  // namespace PJ

QT_BEGIN_NAMESPACE
#define DataRead_iid "facontidavide.PlotJuggler3.DataLoader"
Q_DECLARE_INTERFACE(PJ::DataLoader, DataRead_iid)
QT_END_NAMESPACE

#endif
