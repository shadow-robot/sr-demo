/** -*- mode: c++; c-indent-level: 4; c++-member-init-indent: 8; comment-column: 35; -*-

The above line is usefulin Emacs-like editors

* Copyright 2019 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.

/*
Template for computing statistics on eoPop
============================================
*/

#ifndef _eoSRAutomaticPidTuningStat_h
#define _eoSRAutomaticPidTuningStat_h

// include the base definition of eoInit
#include <utils/eoStat.h>

/**
*  Always write a comment in this format before class definition
*  if you want the class to be documented by Doxygen
*
* ASSUMPTION on the class GenoypeT:
*             it needs to derive from EO (i.e. has a Fitness).
*
* It is assumed that you want to compute a double.
* In case you want something else, then your stat should derive from
*                      eoStat<GenotypeT, T>
*  where class T is the class of the computed statistics
*/
template <class EOT>
class eoSRAutomaticPidTuningStat : public eoStat<EOT, double>
{
public :
    typedef typename EOT::Fitness Fitness;

// START eventually add or modify the anyVariable argument
  /** Ctor - you change the default name of course.
   * @param
   *  _description : inherited from eoValueParam (eoStat is an  from eoVapueParam)
   */
  eoSRAutomaticPidTuningStat(std::string _description = "eoSRAutomaticPidTuningStat ") :
    eoStat<EOT, double>(0.0, _description)
// END eventually add or modify the anyVariable argument
  {
    // START Code of Ctor of an eoMonReelStat object
    // END   Code of Ctor of an eoMonReelStat object
  }

    void operator()(const eoPop<EOT>& _pop){
	double tmpStat(0.);
    // START Code for computing the statistics - in tmpStat
      // tmpStat = blablabla
    // END   Code for computing the statistics
      eoStat<EOT,double>::value() = tmpStat;      // store the stat in the eoParam value() field
    }

  virtual std::string className(void) const { return "eoSRAutomaticPidTuningStat"; }
private :
// START Private data of an eoSRAutomaticPidTuningStat object
  //  varType anyVariable;		   // for example ...
// END   Private data of an eoSRAutomaticPidTuningStat object
};

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
