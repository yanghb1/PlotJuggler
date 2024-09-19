#include "ros_parser.h"
#include "data_tamer_parser/data_tamer_parser.hpp"
#include "PlotJuggler/contrib/fmt/core.h"

using namespace PJ;
using namespace RosMsgParser;

static ROSType quaternion_type(Msg::Quaternion::id());
constexpr double RAD_TO_DEG = 180.0 / M_PI;

static std::unordered_map<uint64_t, DataTamerParser::Schema> _global_data_tamer_schemas;

ParserROS::ParserROS(const std::string& topic_name, const std::string& type_name,
                     const std::string& schema, RosMsgParser::Deserializer* deserializer,
                     PlotDataMapRef& data)
  : MessageParser(topic_name, data)
  , _parser(topic_name, type_name, schema)
  , _deserializer(deserializer)
  , _topic(topic_name)
{
  auto policy =
      clampLargeArray() ? Parser::KEEP_LARGE_ARRAYS : Parser::DISCARD_LARGE_ARRAYS;

  _parser.setMaxArrayPolicy(policy, maxArraySize());

  const auto& root_fields = _parser.getSchema()->root_msg->fields();
  _has_header = !root_fields.empty() && root_fields.front().type().baseName() == "std_"
                                                                                 "msgs/"
                                                                                 "Header";

  using std::placeholders::_1;
  using std::placeholders::_2;
  if (Msg::Empty::id() == type_name)
  {
    _customized_parser = std::bind(&ParserROS::parseEmpty, this, _1, _2);
  }
  else if (Msg::DiagnosticArray::id() == type_name)
  {
    _customized_parser = std::bind(&ParserROS::parseDiagnosticMsg, this, _1, _2);
  }
  else if (Msg::JointState::id() == type_name)
  {
    _customized_parser = std::bind(&ParserROS::parseJointStateMsg, this, _1, _2);
  }
  else if (Msg::TFMessage::id() == type_name)
  {
    _customized_parser = std::bind(&ParserROS::parseTF2Msg, this, _1, _2);
  }
  else if (Msg::DataTamerSchemas::id() == type_name)
  {
    _customized_parser = std::bind(&ParserROS::parseDataTamerSchemas, this, _1, _2);
  }
  else if (Msg::DataTamerSnapshot::id() == type_name)
  {
    _customized_parser = std::bind(&ParserROS::parseDataTamerSnapshot, this, _1, _2);
  }
  else if (Msg::Imu::id() == type_name)
  {
    _customized_parser = std::bind(&ParserROS::parseImu, this, _1, _2);
  }
  else if (Msg::Pose::id() == type_name)
  {
    _customized_parser = std::bind(&ParserROS::parsePose, this, _1, _2);
  }
  else if (Msg::PoseStamped::id() == type_name)
  {
    _customized_parser = std::bind(&ParserROS::parsePoseStamped, this, _1, _2);
  }
  else if (Msg::Odometry::id() == type_name)
  {
    _customized_parser = std::bind(&ParserROS::parseOdometry, this, _1, _2);
  }
  else if (Msg::Transform::id() == type_name)
  {
    _customized_parser = std::bind(&ParserROS::parseTransform, this, _1, _2);
  }
  else if (Msg::TransformStamped::id() == type_name)
  {
    _customized_parser = std::bind(&ParserROS::parseTransformStamped, this, _1, _2);
  }
  else if (Msg::PalStatisticsNames::id() == type_name || type_name == "plotjuggler_msgs/"
                                                                      "StatisticsNames")
  {
    _customized_parser = std::bind(&ParserROS::parsePalStatisticsNames, this, _1, _2);
  }
  else if (Msg::PalStatisticsValues::id() == type_name || type_name == "plotjuggler_msgs/"
                                                                       "StatisticsValues")
  {
    _customized_parser = std::bind(&ParserROS::parsePalStatisticsValues, this, _1, _2);
  }
}

bool ParserROS::parseMessage(const PJ::MessageRef serialized_msg, double& timestamp)
{
  if (_customized_parser)
  {
    _deserializer->init(
        Span<const uint8_t>(serialized_msg.data(), serialized_msg.size()));
    _customized_parser(_topic_name, timestamp);
    return true;
  }

  _parser.deserialize(serialized_msg, &_flat_msg, _deserializer.get());

  if (_has_header && this->useEmbeddedTimestamp())
  {
    double ts = 0;
    if (_deserializer->isROS2())
    {
      auto sec = _flat_msg.value[0].second.convert<double>();
      auto nsec = _flat_msg.value[1].second.convert<double>();
      ts = sec + 1e-9 * nsec;
    }
    else
    {
      ts = _flat_msg.value[1].second.convert<RosMsgParser::Time>().toSec();
    }
    timestamp = (ts > 0) ? ts : timestamp;
  }

  std::string series_name;

  for (const auto& [key, str] : _flat_msg.name)
  {
    key.toStr(series_name);
    StringSeries& data = getStringSeries(series_name);
    data.pushBack({ timestamp, str });
  }

  for (const auto& [key, value] : _flat_msg.value)
  {
    key.toStr(series_name);
    PlotData& data = getSeries(series_name);

    if (!_strict_truncation_check)
    {
      // bypass the truncation check
      if (value.getTypeID() == BuiltinType::INT64)
      {
        data.pushBack({ timestamp, double(value.convert<int64_t>()) });
        continue;
      }
      if (value.getTypeID() == BuiltinType::UINT64)
      {
        data.pushBack({ timestamp, double(value.convert<uint64_t>()) });
        continue;
      }
    }
    try
    {
      data.pushBack({ timestamp, value.convert<double>() });
    }
    catch (RangeException& ex)
    {
      std::string msg = std::string(ex.what());
      if (msg == "Floating point truncated")
      {
        msg += ".\n\nYou can disable this check in:\n"
               "App -> Preferences... -> Behavior -> Parsing";
      }
      throw std::runtime_error(msg);
    }
  }
  return true;
}

void ParserROS::setLargeArraysPolicy(bool clamp, unsigned max_size)
{
  auto policy = clamp ? RosMsgParser::Parser::KEEP_LARGE_ARRAYS :
                        RosMsgParser::Parser::DISCARD_LARGE_ARRAYS;

  _parser.setMaxArrayPolicy(policy, max_size);
  MessageParser::setLargeArraysPolicy(clamp, max_size);
}

//------------------------------------------------------------------------

Msg::Header ParserROS::readHeader(double& timestamp)
{
  Msg::Header header;
  // only ROS1 as the files header.seq
  if (dynamic_cast<ROS_Deserializer*>(_deserializer.get()) != nullptr)
  {
    header.seq = _deserializer->deserializeUInt32();
  }

  header.stamp.sec = _deserializer->deserializeUInt32();
  header.stamp.nanosec = _deserializer->deserializeUInt32();

  const double ts = header.stamp.toSec();
  if (useEmbeddedTimestamp() && ts > 0)
  {
    timestamp = ts;
  }
  _deserializer->deserializeString(header.frame_id);

  return header;
}

void ParserROS::parseHeader(const std::string& prefix, double& timestamp)
{
  const auto header = readHeader(timestamp);

  getSeries(prefix + "/header/stamp").pushBack({ timestamp, header.stamp.toSec() });
  getStringSeries(prefix + "/header/frame_id").pushBack({ timestamp, header.frame_id });
  if (dynamic_cast<ROS_Deserializer*>(_deserializer.get()) != nullptr)
  {
    getSeries(prefix + "/header/seq").pushBack({ timestamp, double(header.seq) });
  }
}

void ParserROS::parseEmpty(const std::string& prefix, double& timestamp)
{
  getSeries(prefix).pushBack({ timestamp, 0 });
}

void ParserROS::parseVector3(const std::string& prefix, double& timestamp)
{
  auto x = _deserializer->deserialize(FLOAT64).convert<double>();
  auto y = _deserializer->deserialize(FLOAT64).convert<double>();
  auto z = _deserializer->deserialize(FLOAT64).convert<double>();
  getSeries(prefix + "/x").pushBack({ timestamp, x });
  getSeries(prefix + "/y").pushBack({ timestamp, y });
  getSeries(prefix + "/z").pushBack({ timestamp, z });
}

void ParserROS::parsePoint(const std::string& prefix, double& timestamp)
{
  auto x = _deserializer->deserialize(FLOAT64).convert<double>();
  auto y = _deserializer->deserialize(FLOAT64).convert<double>();
  auto z = _deserializer->deserialize(FLOAT64).convert<double>();
  getSeries(prefix + "/x").pushBack({ timestamp, x });
  getSeries(prefix + "/y").pushBack({ timestamp, y });
  getSeries(prefix + "/z").pushBack({ timestamp, z });
}

void ParserROS::parseQuaternion(const std::string& prefix, double& timestamp)
{
  PJ::Msg::Quaternion quat;
  quat.x = _deserializer->deserialize(FLOAT64).convert<double>();
  quat.y = _deserializer->deserialize(FLOAT64).convert<double>();
  quat.z = _deserializer->deserialize(FLOAT64).convert<double>();
  quat.w = _deserializer->deserialize(FLOAT64).convert<double>();
  getSeries(prefix + "/x").pushBack({ timestamp, quat.x });
  getSeries(prefix + "/y").pushBack({ timestamp, quat.y });
  getSeries(prefix + "/z").pushBack({ timestamp, quat.z });
  getSeries(prefix + "/w").pushBack({ timestamp, quat.w });

  auto rpy = Msg::QuaternionToRPY(quat);
  getSeries(prefix + "/roll").pushBack({ timestamp, rpy.roll });
  getSeries(prefix + "/pitch").pushBack({ timestamp, rpy.pitch });
  getSeries(prefix + "/yaw").pushBack({ timestamp, rpy.yaw });
}

void ParserROS::parseTwist(const std::string& prefix, double& timestamp)
{
  parseVector3(prefix + "/linear", timestamp);
  parseVector3(prefix + "/angular", timestamp);
}

void ParserROS::parseTwistWithCovariance(const std::string& prefix, double& timestamp)
{
  parseTwist(prefix + "/twist", timestamp);
  parseCovariance<6>(prefix + "/covariance", timestamp);
}

void ParserROS::parseTransform(const std::string& prefix, double& timestamp)
{
  parsePoint(prefix + "/translation", timestamp);
  parseQuaternion(prefix + "/rotation", timestamp);
}

void ParserROS::parseTransformStamped(const std::string& prefix, double& timestamp)
{
  parseHeader(prefix + "/header", timestamp);

  std::string child_frame_id;
  _deserializer->deserializeString(child_frame_id);
  getStringSeries(prefix + "/child_frame_id").pushBack({ timestamp, child_frame_id });

  parseTransform(prefix + "/transform", timestamp);
}

void ParserROS::parsePose(const std::string& prefix, double& timestamp)
{
  parseVector3(prefix + "/position", timestamp);
  parseQuaternion(prefix + "/orientation", timestamp);
}

void ParserROS::parsePoseStamped(const std::string& prefix, double& timestamp)
{
  parseHeader(prefix + "/header", timestamp);
  parsePose(prefix + "/pose", timestamp);
}

void ParserROS::parsePoseWithCovariance(const std::string& prefix, double& timestamp)
{
  parsePose(prefix + "/pose", timestamp);
  parseCovariance<6>(prefix + "/covariance", timestamp);
}

void ParserROS::parseImu(const std::string& prefix, double& timestamp)
{
  parseHeader(prefix + "/header", timestamp);

  parseQuaternion(prefix + "/orientation", timestamp);
  parseCovariance<3>(prefix + "/orientation_covariance", timestamp);

  parseVector3(prefix + "/angular_velocity", timestamp);
  parseCovariance<3>(prefix + "/angular_velocity_covariance", timestamp);

  parseVector3(prefix + "/linear_acceleration", timestamp);
  parseCovariance<3>(prefix + "/linear_acceleration_covariance", timestamp);
}

void ParserROS::parseOdometry(const std::string& prefix, double& timestamp)
{
  parseHeader(prefix + "/header", timestamp);
  std::string child_frame_id;
  _deserializer->deserializeString(child_frame_id);
  getStringSeries(prefix + "/child_frame_id").pushBack({ timestamp, child_frame_id });
  parsePoseWithCovariance(prefix + "/pose", timestamp);
  parseTwistWithCovariance(prefix + "/twist", timestamp);
}

void ParserROS::parseDiagnosticMsg(const std::string& prefix, double& timestamp)
{
  thread_local Msg::DiagnosticArray msg;

  parseHeader(prefix + "/header", timestamp);

  size_t status_count = _deserializer->deserializeUInt32();
  msg.status.resize(status_count);

  for (size_t st = 0; st < status_count; st++)
  {
    auto& status = msg.status[st];
    status.level = _deserializer->deserialize(BYTE).convert<uint8_t>();
    _deserializer->deserializeString(status.name);
    _deserializer->deserializeString(status.message);
    _deserializer->deserializeString(status.hardware_id);

    status.key_value.clear();

    size_t key_value_count = _deserializer->deserializeUInt32();
    std::string key;
    std::string value_str;
    for (size_t kv = 0; kv < key_value_count; kv++)
    {
      _deserializer->deserializeString(key);
      _deserializer->deserializeString(value_str);
      status.key_value.push_back({ key, value_str });
    }
  }

  //------ Now create the series --------

  std::string series_name;

  for (const auto& status : msg.status)
  {
    for (const auto& kv : status.key_value)
    {
      if (status.hardware_id.empty())
      {
        series_name = fmt::format("{}/{}/{}", prefix, status.name, kv.first);
      }
      else
      {
        series_name =
            fmt::format("{}/{}/{}/{}", prefix, status.hardware_id, status.name, kv.first);
      }

      bool ok;
      double value = QString::fromStdString(kv.second).toDouble(&ok);

      if (ok)
      {
        getSeries(series_name).pushBack({ timestamp, value });
      }
      else
      {
        getStringSeries(series_name).pushBack({ timestamp, kv.second });
      }
    }
  }
}

void ParserROS::parseJointStateMsg(const std::string& prefix, double& timestamp)
{
  thread_local Msg::JointState msg;

  msg.name.clear();
  msg.position.clear();
  msg.velocity.clear();
  msg.effort.clear();

  parseHeader(prefix, timestamp);

  size_t name_size = _deserializer->deserializeUInt32();
  if (name_size > 0)
  {
    msg.name.resize(name_size);
    for (auto& name : msg.name)
    {
      _deserializer->deserializeString(name);
    }
  }

  //-----------
  size_t pos_size = _deserializer->deserializeUInt32();
  if (pos_size > 0)
  {
    msg.position.resize(pos_size);
    for (auto& pos : msg.position)
    {
      pos = _deserializer->deserialize(FLOAT64).convert<double>();
    }
  }
  //-----------
  size_t vel_size = _deserializer->deserializeUInt32();
  if (vel_size > 0)
  {
    msg.velocity.resize(vel_size);
    for (auto& vel : msg.velocity)
    {
      vel = _deserializer->deserialize(FLOAT64).convert<double>();
    }
  }
  //-----------
  size_t eff_size = _deserializer->deserializeUInt32();
  if (eff_size > 0)
  {
    msg.effort.resize(eff_size);
    for (auto& eff : msg.effort)
    {
      eff = _deserializer->deserialize(FLOAT64).convert<double>();
    }
  }
  //---------------------------
  std::string series_name;
  for (size_t i = 0; i < std::min(name_size, pos_size); i++)
  {
    series_name = fmt::format("{}/{}/position", _topic, msg.name[i]);
    getSeries(series_name).pushBack({ timestamp, msg.position[i] });
  }
  for (size_t i = 0; i < std::min(name_size, vel_size); i++)
  {
    series_name = fmt::format("{}/{}/velocity", _topic, msg.name[i]);
    getSeries(series_name).pushBack({ timestamp, msg.velocity[i] });
  }
  for (size_t i = 0; i < std::min(name_size, eff_size); i++)
  {
    series_name = fmt::format("{}/{}/effort", _topic, msg.name[i]);
    getSeries(series_name).pushBack({ timestamp, msg.effort[i] });
  }
}

void ParserROS::parseTF2Msg(const std::string& prefix, double& timestamp)
{
  const size_t transform_size = _deserializer->deserializeUInt32();

  if (transform_size == 0)
  {
    return;
  }

  for (size_t i = 0; i < transform_size; i++)
  {
    const auto header = readHeader(timestamp);
    std::string child_frame_id;
    _deserializer->deserializeString(child_frame_id);

    std::string new_prefix;
    if (header.frame_id.empty())
    {
      new_prefix = fmt::format("{}/{}", prefix, child_frame_id);
    }
    else
    {
      new_prefix = fmt::format("{}/{}/{}", prefix, header.frame_id, child_frame_id);
    }
    parseTransform(new_prefix, timestamp);
  }
}

void ParserROS::parseDataTamerSchemas(const std::string& prefix, double& timestamp)
{
  const size_t vector_size = _deserializer->deserializeUInt32();

  for (size_t i = 0; i < vector_size; i++)
  {
    DataTamerParser::Schema schema;
    schema.hash = _deserializer->deserialize(BuiltinType::UINT64).convert<uint64_t>();
    std::string channel_name;
    _deserializer->deserializeString(channel_name);
    std::string schema_text;
    _deserializer->deserializeString(schema_text);

    auto dt_schema = DataTamerParser::BuilSchemaFromText(schema_text);
    dt_schema.channel_name = channel_name;
    _global_data_tamer_schemas.insert({ dt_schema.hash, dt_schema });
  }
}

void ParserROS::parseDataTamerSnapshot(const std::string& prefix, double& timestamp)
{
  DataTamerParser::SnapshotView snapshot;

  snapshot.timestamp =
      _deserializer->deserialize(BuiltinType::UINT64).convert<uint64_t>();
  snapshot.schema_hash =
      _deserializer->deserialize(BuiltinType::UINT64).convert<uint64_t>();

  auto active_mask = _deserializer->deserializeByteSequence();
  snapshot.active_mask = { active_mask.data(), active_mask.size() };

  auto payload = _deserializer->deserializeByteSequence();
  snapshot.payload = { payload.data(), payload.size() };

  auto it = _global_data_tamer_schemas.find(snapshot.schema_hash);
  if (it == _global_data_tamer_schemas.end())
  {
    return;
  }
  const auto& dt_schema = it->second;

  const auto toDouble = [](const auto& value) { return static_cast<double>(value); };

  auto callback = [&](const std::string& name_field,
                      const DataTamerParser::VarNumber& value) {
    double timestamp = double(snapshot.timestamp) * 1e-9;
    auto name = fmt::format("{}/{}/{}", _topic_name, dt_schema.channel_name, name_field);
    getSeries(name).pushBack({ timestamp, std::visit(toDouble, value) });
  };

  DataTamerParser::ParseSnapshot(dt_schema, snapshot, callback);
}

static std::unordered_map<uint32_t, std::vector<std::string>> _pal_statistics_names;

void ParserROS::parsePalStatisticsNames(const std::string& prefix, double& timestamp)
{
  const auto header = readHeader(timestamp);
  std::vector<std::string> names;
  const size_t vector_size = _deserializer->deserializeUInt32();
  names.resize(vector_size);
  for (auto& name : names)
  {
    _deserializer->deserializeString(name);
  }
  uint32_t names_version = _deserializer->deserializeUInt32();
  _pal_statistics_names[names_version] = std::move(names);
}

void ParserROS::parsePalStatisticsValues(const std::string& prefix, double& timestamp)
{
  const auto header = readHeader(timestamp);
  std::vector<double> values;
  const size_t vector_size = _deserializer->deserializeUInt32();
  values.resize(vector_size);

  for (auto& value : values)
  {
    value = _deserializer->deserialize(BuiltinType::FLOAT64).convert<double>();
  }
  uint32_t names_version = _deserializer->deserializeUInt32();
  auto it = _pal_statistics_names.find(names_version);
  if (it != _pal_statistics_names.end())
  {
    const auto& names = it->second;
    const size_t N = std::min(names.size(), values.size());
    for (size_t i = 0; i < N; i++)
    {
      auto& series = getSeries(fmt::format("{}/{}", prefix, names[i]));
      series.pushBack({ timestamp, values[i] });
    }
  }
}
