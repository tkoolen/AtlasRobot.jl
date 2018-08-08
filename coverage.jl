import Pkg
Pkg.add("Coverage")
using Coverage
Codecov.submit(Codecov.process_folder())
