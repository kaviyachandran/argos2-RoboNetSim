set(chipmunk_HEADERS
  include/cpShape.h
  include/cpBody.h
  include/chipmunk_unsafe.h
  include/cpSpace.h
  include/constraints/cpGearJoint.h
  include/constraints/cpSlideJoint.h
  include/constraints/cpPivotJoint.h
  include/constraints/cpDampedRotarySpring.h
  include/constraints/cpSimpleMotor.h
  include/constraints/cpGrooveJoint.h
  include/constraints/cpRotaryLimitJoint.h
  include/constraints/cpRatchetJoint.h
  include/constraints/cpConstraint.h
  include/constraints/util.h
  include/constraints/cpPinJoint.h
  include/constraints/cpDampedSpring.h
  include/cpVect.h
  include/chipmunk_private.h
  include/chipmunk_types.h
  include/cpBB.h
  include/cpArbiter.h
  include/cpSpatialIndex.h
  include/chipmunk_ffi.h
  include/cpPolyShape.h
  include/chipmunk.h
  )

set(chipmunk_IMPLEMENTATIONS
  src/cpShape.c
  src/cpBody.c
  src/cpSpaceComponent.c
  src/cpBBTree.c
  src/cpSpace.c
  src/cpSpaceStep.c
  src/cpSweep1D.c
  src/constraints/cpGearJoint.c
  src/constraints/cpSlideJoint.c
  src/constraints/cpPivotJoint.c
  src/constraints/cpDampedRotarySpring.c
  src/constraints/cpSimpleMotor.c
  src/constraints/cpGrooveJoint.c
  src/constraints/cpRotaryLimitJoint.c
  src/constraints/cpRatchetJoint.c
  src/constraints/cpConstraint.c
  src/constraints/cpPinJoint.c
  src/constraints/cpDampedSpring.c
  src/cpVect.c
  src/cpBB.c
  src/cpSpaceQuery.c
  src/cpArbiter.c
  src/cpArray.c
  src/cpHashSet.c
  src/cpSpatialIndex.c
  src/cpPolyShape.c
  src/cpCollision.c
  src/cpSpaceHash.c
  src/chipmunk.c
  )

include_directories(${CMAKE_SOURCE_DIR}/physics_engines/dynamics2d/chipmunk-physics/include)
add_library(chipmunk SHARED ${chipmunk_IMPLEMENTATIONS})

install(TARGETS chipmunk
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos2
  ARCHIVE DESTINATION lib/argos2)
