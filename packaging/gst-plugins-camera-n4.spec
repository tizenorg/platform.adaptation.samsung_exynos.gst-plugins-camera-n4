Name:       gst-plugins-camera-n4
Summary:    GStreamer codec plugins package for N4
Version:    0.0.1
Release:    0
Group:      Multimedia/Framework
License:    LGPL-2.1+ and Apache-2.0
Source0:    %{name}-%{version}.tar.gz
ExclusiveArch:  %arm aarch64
BuildRequires:  pkgconfig(gstreamer-plugins-base-1.0)
BuildRequires:  pkgconfig(mm-common)
BuildRequires:  pkgconfig(libtbm)
BuildRequires:  pkgconfig(libdrm)
BuildRequires:  pkgconfig(libdrm_exynos)
BuildRequires:  gstreamer-devel
BuildConflicts:  linux-glibc-devel
%ifarch %{arm}
BuildRequires:  arm-trats2-linux-kernel-headers >= 3.10
%else
BuildRequires:  arm64-juno-linux-kernel-headers >= 3.15
%endif

%description
GStreamer codec plugins package for N4
 GStreamer is a streaming media framework, based on graphs of filters
 which operate on media data. Multimedia Framework using this plugins
 library can encode and decode video, audio, and speech..

%prep
%setup -q

%build
sh ./autogen.sh
%configure --disable-static
make %{?jobs:-j%jobs}

%install
rm -rf %{buildroot}
mkdir -p %{buildroot}/usr/share/license
cp LICENSE.LGPLv2 %{buildroot}/usr/share/license/%{name}
cat LICENSE.APLv2 >> %{buildroot}/usr/share/license/%{name}
%make_install

%files
%manifest gst-plugins-camera-n4.manifest
%defattr(-,root,root,-)
%{_libdir}/gstreamer-1.0/lib*.so*
%{_datadir}/license/%{name}

