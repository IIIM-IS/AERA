#pragma once

#include "tcp_data_message.pb.h"

namespace tcp_io_device {
  /**
  * MetaData is a class to store the meta-data of messages. Especially used for storing of available commands and their descriptions.
  * Additionally gives access to convenience funtions, like VariableDescription message parsing.
  */
  class MetaData {
    friend class MsgData;

  protected:
    int entity_id_;
    int id_;
    VariableDescription_DataType type_;
    size_t type_size_;
    uint64_t data_size_ = 0;
    uint64_t data_length_ = 0;
    std::vector<uint64_t> dimensions_;

  public:
    /**
    * Constructor for MetaData objects. Converts the passed VariableDescription message and creates a MetaData object from it.
    * \param meta_data The VariableDescription used to convert to a MetaData object.
    */
    MetaData(const VariableDescription* meta_data) {
      setMetaData(meta_data->entityid(),
        meta_data->id(),
        meta_data->datatype(),
        std::vector<uint64_t>(meta_data->dimensions().begin(), meta_data->dimensions().end()));
    }

    /**
    * Returns the id of the entity for which this description is used.
    */
    int getEntityID() {
      return entity_id_;
    }

    /**
    * Returns the id of the property for which this description is used.
    */
    int getID() {
      return id_;
    }

    /**
    * Returns the full length of the data assigned to this MetaData object
    * Number of data parts (e.g. double values) * the data size of each part (e.g. 8)
    */
    uint64_t getDataLength() { return data_length_; }

    /**
    * Returns the number of bytes used to store a data object (e.g. 8 for double)
    */
    uint64_t getDataSize() { return data_size_; }

    /**
    * Returns the dimensions of the data. E.g. [1920, 1080] for a full HD image.
    */
    std::vector<uint64_t> getDimensions() { return dimensions_; }

    /**
    * Returns the type of the corresponding message.
    */
    VariableDescription_DataType getType() { return type_; }


    /**
    * Sets the fields of the MetaData object.
    * \param entity_id The id of the entity as received in the setup message.
    * \param id The id of the property as received in the setup message.
    * \param t The data type of the data (e.g. DOUBLE, INT, or similar).
    * \param d The dimensions of the data.
    */
    void setMetaData(int entity_id, int id, VariableDescription_DataType t, std::vector<uint64_t> d) {
      entity_id_ = entity_id;
      id_ = id;
      type_ = t;
      dimensions_ = d;
      switch (t)
      {
      case 0:
        type_size_ = 8;
        break;
      case 3:
        type_size_ = 8;
        break;
      default:
        type_size_ = 1;
        break;
      }
      data_size_ = 1;
      data_length_ = 1;
      for (int i = 0; i < dimensions_.size(); ++i) {
        data_size_ *= dimensions_[i] * type_size_;
        data_length_ *= dimensions_[i];
      }
    }
  };


  /**
  * Class to store a DataMessage including a MetaData object and a string for the bytes of data.
  */
  class MsgData {
  private:
    MetaData meta_data_;
    std::string data_;
  public:

    /**
    * Constructor for MsgData objects. Converts a ProtoVariable message to a MsgData object.
    * \param msg The message used to convert and create a MsgData object from.
    */
    MsgData(const ProtoVariable* msg) : meta_data_(&(msg->metadata())) {
      setData(msg->data());
    }


    /**
    * Setter for the data of the message. Data is stored as a byte representation in form of a std::string.
    * \param d The byte representation of the data in form of a std::string.
    */
    void setData(std::string d) {
      data_ = d;
    }


    /**
    * Returns the MetaData object corresponding to this MsgData.
    */
    MetaData getMetaData() {
      return meta_data_;
    }


    /**
    * Casts the data from the byte representation stored as a string to the template type.
    */
    template <typename T> std::vector<T> getData() {
      T a;
      std::vector<T> values;
      for (int i = 0; i < meta_data_.data_size_; i += meta_data_.type_size_) {
        char* pos = &data_[i];
        memcpy(&a, pos, meta_data_.type_size_);
        values.push_back(a);
      }
      return values;
    }
  };


}