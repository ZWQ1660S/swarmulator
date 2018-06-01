// Copyright (c) 2005-2014 Code Synthesis Tools CC
//
// This program was generated by CodeSynthesis XSD, an XML Schema to
// C++ data binding compiler.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
//
// In addition, as a special exception, Code Synthesis Tools CC gives
// permission to link this program with the Xerces-C++ library (or with
// modified versions of Xerces-C++ that use the same license as Xerces-C++),
// and distribute linked combinations including the two. You must obey
// the GNU General Public License version 2 in all respects for all of
// the code used other than Xerces-C++. If you modify this copy of the
// program, you may extend this exception to your version of the program,
// but you are not obligated to do so. If you do not wish to do so, delete
// this exception statement from your version.
//
// Furthermore, Code Synthesis Tools CC makes a special exception for
// the Free/Libre and Open Source Software (FLOSS) which is described
// in the accompanying FLOSSE file.
//

// Begin prologue.
//
//
// End prologue.

#include <xsd/cxx/pre.hxx>

#include "xml_layout.hxx"

// parameters_t
// 

const parameters_t::greeting_type& parameters_t::
greeting () const
{
  return this->greeting_.get ();
}

parameters_t::greeting_type& parameters_t::
greeting ()
{
  return this->greeting_.get ();
}

void parameters_t::
greeting (const greeting_type& x)
{
  this->greeting_.set (x);
}

void parameters_t::
greeting (::std::auto_ptr< greeting_type > x)
{
  this->greeting_.set (x);
}

const parameters_t::name_sequence& parameters_t::
name () const
{
  return this->name_;
}

parameters_t::name_sequence& parameters_t::
name ()
{
  return this->name_;
}

void parameters_t::
name (const name_sequence& s)
{
  this->name_ = s;
}


#include <xsd/cxx/xml/dom/parsing-source.hxx>

// parameters_t
//

parameters_t::
parameters_t (const greeting_type& greeting)
: ::xml_schema::type (),
  greeting_ (greeting, this),
  name_ (this)
{
}

parameters_t::
parameters_t (const parameters_t& x,
              ::xml_schema::flags f,
              ::xml_schema::container* c)
: ::xml_schema::type (x, f, c),
  greeting_ (x.greeting_, f, this),
  name_ (x.name_, f, this)
{
}

parameters_t::
parameters_t (const ::xercesc::DOMElement& e,
              ::xml_schema::flags f,
              ::xml_schema::container* c)
: ::xml_schema::type (e, f | ::xml_schema::flags::base, c),
  greeting_ (this),
  name_ (this)
{
  if ((f & ::xml_schema::flags::base) == 0)
  {
    ::xsd::cxx::xml::dom::parser< char > p (e, true, false, false);
    this->parse (p, f);
  }
}

void parameters_t::
parse (::xsd::cxx::xml::dom::parser< char >& p,
       ::xml_schema::flags f)
{
  for (; p.more_content (); p.next_content (false))
  {
    const ::xercesc::DOMElement& i (p.cur_element ());
    const ::xsd::cxx::xml::qualified_name< char > n (
      ::xsd::cxx::xml::dom::name< char > (i));

    // greeting
    //
    if (n.name () == "greeting" && n.namespace_ ().empty ())
    {
      ::std::auto_ptr< greeting_type > r (
        greeting_traits::create (i, f, this));

      if (!greeting_.present ())
      {
        this->greeting_.set (r);
        continue;
      }
    }

    // name
    //
    if (n.name () == "name" && n.namespace_ ().empty ())
    {
      ::std::auto_ptr< name_type > r (
        name_traits::create (i, f, this));

      this->name_.push_back (r);
      continue;
    }

    break;
  }

  if (!greeting_.present ())
  {
    throw ::xsd::cxx::tree::expected_element< char > (
      "greeting",
      "");
  }
}

parameters_t* parameters_t::
_clone (::xml_schema::flags f,
        ::xml_schema::container* c) const
{
  return new class parameters_t (*this, f, c);
}

parameters_t& parameters_t::
operator= (const parameters_t& x)
{
  if (this != &x)
  {
    static_cast< ::xml_schema::type& > (*this) = x;
    this->greeting_ = x.greeting_;
    this->name_ = x.name_;
  }

  return *this;
}

parameters_t::
~parameters_t ()
{
}

#include <istream>
#include <xsd/cxx/xml/sax/std-input-source.hxx>
#include <xsd/cxx/tree/error-handler.hxx>

::std::auto_ptr< ::parameters_t >
parameters (const ::std::string& u,
            ::xml_schema::flags f,
            const ::xml_schema::properties& p)
{
  ::xsd::cxx::xml::auto_initializer i (
    (f & ::xml_schema::flags::dont_initialize) == 0,
    (f & ::xml_schema::flags::keep_dom) == 0);

  ::xsd::cxx::tree::error_handler< char > h;

  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::xsd::cxx::xml::dom::parse< char > (
      u, h, p, f));

  h.throw_if_failed< ::xsd::cxx::tree::parsing< char > > ();

  return ::std::auto_ptr< ::parameters_t > (
    ::parameters (
      d, f | ::xml_schema::flags::own_dom, p));
}

::std::auto_ptr< ::parameters_t >
parameters (const ::std::string& u,
            ::xml_schema::error_handler& h,
            ::xml_schema::flags f,
            const ::xml_schema::properties& p)
{
  ::xsd::cxx::xml::auto_initializer i (
    (f & ::xml_schema::flags::dont_initialize) == 0,
    (f & ::xml_schema::flags::keep_dom) == 0);

  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::xsd::cxx::xml::dom::parse< char > (
      u, h, p, f));

  if (!d.get ())
    throw ::xsd::cxx::tree::parsing< char > ();

  return ::std::auto_ptr< ::parameters_t > (
    ::parameters (
      d, f | ::xml_schema::flags::own_dom, p));
}

::std::auto_ptr< ::parameters_t >
parameters (const ::std::string& u,
            ::xercesc::DOMErrorHandler& h,
            ::xml_schema::flags f,
            const ::xml_schema::properties& p)
{
  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::xsd::cxx::xml::dom::parse< char > (
      u, h, p, f));

  if (!d.get ())
    throw ::xsd::cxx::tree::parsing< char > ();

  return ::std::auto_ptr< ::parameters_t > (
    ::parameters (
      d, f | ::xml_schema::flags::own_dom, p));
}

::std::auto_ptr< ::parameters_t >
parameters (::std::istream& is,
            ::xml_schema::flags f,
            const ::xml_schema::properties& p)
{
  ::xsd::cxx::xml::auto_initializer i (
    (f & ::xml_schema::flags::dont_initialize) == 0,
    (f & ::xml_schema::flags::keep_dom) == 0);

  ::xsd::cxx::xml::sax::std_input_source isrc (is);
  return ::parameters (isrc, f, p);
}

::std::auto_ptr< ::parameters_t >
parameters (::std::istream& is,
            ::xml_schema::error_handler& h,
            ::xml_schema::flags f,
            const ::xml_schema::properties& p)
{
  ::xsd::cxx::xml::auto_initializer i (
    (f & ::xml_schema::flags::dont_initialize) == 0,
    (f & ::xml_schema::flags::keep_dom) == 0);

  ::xsd::cxx::xml::sax::std_input_source isrc (is);
  return ::parameters (isrc, h, f, p);
}

::std::auto_ptr< ::parameters_t >
parameters (::std::istream& is,
            ::xercesc::DOMErrorHandler& h,
            ::xml_schema::flags f,
            const ::xml_schema::properties& p)
{
  ::xsd::cxx::xml::sax::std_input_source isrc (is);
  return ::parameters (isrc, h, f, p);
}

::std::auto_ptr< ::parameters_t >
parameters (::std::istream& is,
            const ::std::string& sid,
            ::xml_schema::flags f,
            const ::xml_schema::properties& p)
{
  ::xsd::cxx::xml::auto_initializer i (
    (f & ::xml_schema::flags::dont_initialize) == 0,
    (f & ::xml_schema::flags::keep_dom) == 0);

  ::xsd::cxx::xml::sax::std_input_source isrc (is, sid);
  return ::parameters (isrc, f, p);
}

::std::auto_ptr< ::parameters_t >
parameters (::std::istream& is,
            const ::std::string& sid,
            ::xml_schema::error_handler& h,
            ::xml_schema::flags f,
            const ::xml_schema::properties& p)
{
  ::xsd::cxx::xml::auto_initializer i (
    (f & ::xml_schema::flags::dont_initialize) == 0,
    (f & ::xml_schema::flags::keep_dom) == 0);

  ::xsd::cxx::xml::sax::std_input_source isrc (is, sid);
  return ::parameters (isrc, h, f, p);
}

::std::auto_ptr< ::parameters_t >
parameters (::std::istream& is,
            const ::std::string& sid,
            ::xercesc::DOMErrorHandler& h,
            ::xml_schema::flags f,
            const ::xml_schema::properties& p)
{
  ::xsd::cxx::xml::sax::std_input_source isrc (is, sid);
  return ::parameters (isrc, h, f, p);
}

::std::auto_ptr< ::parameters_t >
parameters (::xercesc::InputSource& i,
            ::xml_schema::flags f,
            const ::xml_schema::properties& p)
{
  ::xsd::cxx::tree::error_handler< char > h;

  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::xsd::cxx::xml::dom::parse< char > (
      i, h, p, f));

  h.throw_if_failed< ::xsd::cxx::tree::parsing< char > > ();

  return ::std::auto_ptr< ::parameters_t > (
    ::parameters (
      d, f | ::xml_schema::flags::own_dom, p));
}

::std::auto_ptr< ::parameters_t >
parameters (::xercesc::InputSource& i,
            ::xml_schema::error_handler& h,
            ::xml_schema::flags f,
            const ::xml_schema::properties& p)
{
  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::xsd::cxx::xml::dom::parse< char > (
      i, h, p, f));

  if (!d.get ())
    throw ::xsd::cxx::tree::parsing< char > ();

  return ::std::auto_ptr< ::parameters_t > (
    ::parameters (
      d, f | ::xml_schema::flags::own_dom, p));
}

::std::auto_ptr< ::parameters_t >
parameters (::xercesc::InputSource& i,
            ::xercesc::DOMErrorHandler& h,
            ::xml_schema::flags f,
            const ::xml_schema::properties& p)
{
  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::xsd::cxx::xml::dom::parse< char > (
      i, h, p, f));

  if (!d.get ())
    throw ::xsd::cxx::tree::parsing< char > ();

  return ::std::auto_ptr< ::parameters_t > (
    ::parameters (
      d, f | ::xml_schema::flags::own_dom, p));
}

::std::auto_ptr< ::parameters_t >
parameters (const ::xercesc::DOMDocument& doc,
            ::xml_schema::flags f,
            const ::xml_schema::properties& p)
{
  if (f & ::xml_schema::flags::keep_dom)
  {
    ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
      static_cast< ::xercesc::DOMDocument* > (doc.cloneNode (true)));

    return ::std::auto_ptr< ::parameters_t > (
      ::parameters (
        d, f | ::xml_schema::flags::own_dom, p));
  }

  const ::xercesc::DOMElement& e (*doc.getDocumentElement ());
  const ::xsd::cxx::xml::qualified_name< char > n (
    ::xsd::cxx::xml::dom::name< char > (e));

  if (n.name () == "parameters" &&
      n.namespace_ () == "")
  {
    ::std::auto_ptr< ::parameters_t > r (
      ::xsd::cxx::tree::traits< ::parameters_t, char >::create (
        e, f, 0));
    return r;
  }

  throw ::xsd::cxx::tree::unexpected_element < char > (
    n.name (),
    n.namespace_ (),
    "parameters",
    "");
}

::std::auto_ptr< ::parameters_t >
parameters (::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d,
            ::xml_schema::flags f,
            const ::xml_schema::properties&)
{
  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > c (
    ((f & ::xml_schema::flags::keep_dom) &&
     !(f & ::xml_schema::flags::own_dom))
    ? static_cast< ::xercesc::DOMDocument* > (d->cloneNode (true))
    : 0);

  ::xercesc::DOMDocument& doc (c.get () ? *c : *d);
  const ::xercesc::DOMElement& e (*doc.getDocumentElement ());

  const ::xsd::cxx::xml::qualified_name< char > n (
    ::xsd::cxx::xml::dom::name< char > (e));

  if (f & ::xml_schema::flags::keep_dom)
    doc.setUserData (::xml_schema::dom::tree_node_key,
                     (c.get () ? &c : &d),
                     0);

  if (n.name () == "parameters" &&
      n.namespace_ () == "")
  {
    ::std::auto_ptr< ::parameters_t > r (
      ::xsd::cxx::tree::traits< ::parameters_t, char >::create (
        e, f, 0));
    return r;
  }

  throw ::xsd::cxx::tree::unexpected_element < char > (
    n.name (),
    n.namespace_ (),
    "parameters",
    "");
}

#include <xsd/cxx/post.hxx>

// Begin epilogue.
//
//
// End epilogue.

