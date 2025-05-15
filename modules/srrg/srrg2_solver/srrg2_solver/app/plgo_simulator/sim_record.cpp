#include "sim_record.h"

namespace srrg2_solver {
  using namespace std;
  using namespace srrg2_core;
  void SimRecord::serialize(ObjectData& odata, IdContext& context)  {
    odata.setInt("epoch", epoch);
    ArrayData* adata=new ArrayData;
    for (size_t i=0; i<vars.size(); ++i)
      adata->add(vars[i]);
    odata.setField("variables", adata);
    adata=new ArrayData;
    for (size_t i=0; i<factors.size(); ++i)
      adata->add(factors[i]);
    odata.setField("factors", adata);
  }
  
  void SimRecord::deserialize(ObjectData& odata, IdContext& context)  {
    epoch = odata.getInt("epoch");
    ValueData* array=odata.getField("variables");
    ArrayData& v_data=dynamic_cast<ArrayData&>(*array);
    vars.resize(v_data.size());
    for (size_t i=0; i<vars.size(); ++i)
      vars[i]=v_data[i].getUnsignedInt();
    
    array=odata.getField("factors");
    ArrayData& f_data=dynamic_cast<ArrayData&>(*array);
    factors.resize(f_data.size());
    for (size_t i=0; i<factors.size(); ++i)
      factors[i]=f_data[i].getUnsignedInt();
  }
}
