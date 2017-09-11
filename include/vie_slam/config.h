namespace vie_slam {

class Config {
private:
  static std::shared_ptr<Config> config_;
  cv::FileStorage file_;

  Config (){}

public:
  virtual ~Config ();

  static void setParameterFile(const std::string& filename); 

  template<typename T>
  static T get(const std::string& key){
    return T(Config::config_->file_[key]);
  }
};

} /* vie_slam */
