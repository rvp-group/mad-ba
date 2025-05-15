namespace srrg2_solver {
    template <typename VariableType1_, typename VariableType2_ >
    Eigen::Matrix<float, VariableType1_::PerturbationDim, VariableType2_::PerturbationDim>
    Solver::extractFisherInformationBlock(const VariableType1_& variable1_,
                                          const VariableType2_& variable2_) const {
      bool transposed;
      const MatrixBlockBase* m=_extractFisherInformationBlock(transposed, variable1_, variable2_);
      if (! m) {
        throw std::runtime_error("Solver| extractFisherInformationBlock, no block for vars");
      }
      Eigen::Matrix<float, VariableType1_::PerturbationDim, VariableType2_::PerturbationDim> dest;
      MatrixBlockAlias alias(VariableType1_::PerturbationDim,
                             VariableType2_::PerturbationDim,
                             &dest(0,0));
      if (transposed)
        m->transposeTo(&alias);
      else
        m->copyTo(&alias);
      return dest;
    }

}
