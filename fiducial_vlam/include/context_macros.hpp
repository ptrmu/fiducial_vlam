#ifndef FIDUCIAL_VLAM_CONTEXT_MACROS_HPP
#define FIDUCIAL_VLAM_CONTEXT_MACROS_HPP

#define CXT_PARAM_FIELD_DEF(n, d, t) t n##_ = d;

#define CXT_MEMBER_FIELD_DEF(n, t) t n##_;

#define CXT_PARAM_LOAD_PARAM(n, d, t) node.get_parameter<t>(#n, n##_);

#endif //FIDUCIAL_VLAM_CONTEXT_MACROS_HPP
